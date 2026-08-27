// test_state_dump.cpp — 계측(GaitAux)이 «컨트롤러가 실제로 쓴 값» 을 그대로 나르는지,
// 그리고 열 이름과 값의 개수가 맞는지. 계측은 기본 꺼짐이라 사람 눈에 안 띄고 낡는다 —
// 그래서 실기 프리플라이트(verify_deploy.py `_TESTS_PURE`)가 매번 이걸 돌린다.
//
//   cd deploy/robots/g1/tests && g++ -std=gnu++17 -O3 -DNDEBUG -I../include \
//       test_state_dump.cpp -o /tmp/tsd && /tmp/tsd
#include "MaskedLocoController.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>

static int fail = 0;
static void chk(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++fail; }
}
static void chkf(float got, float want, const char* what, float tol = 1e-6f) {
    if (!(std::fabs(got - want) <= tol)) {
        std::printf("FAIL %s: got %.7f want %.7f\n", what, got, want); ++fail;
    }
}
static int commas(const std::string& s) { return (int)std::count(s.begin(), s.end(), ','); }

// GaitAux 를 한 줄 써서 문자열로 받는다.
static std::string render(const GaitAux& g) {
    char buf[512];
    std::FILE* f = fmemopen(buf, sizeof buf, "w");
    g.write(f);
    std::fclose(f);            // fmemopen 은 close 시 NUL 을 붙인다
    return std::string(buf);
}

int main() {
    // ── ① 열 이름 개수 == 값 개수 ────────────────────────────────────────────
    // 필드를 늘리면서 header() 나 write() 중 하나만 고치면 여기서 걸린다.
    {
        GaitAux g;
        const int n_head = commas(GaitAux::header());
        const int n_row  = commas(render(g));
        if (n_head != n_row)
            std::printf("FAIL 열 개수: header %d개 vs row %d개 "
                        "(GaitAux::header() 와 write() 중 한쪽만 고쳤다)\n", n_head, n_row);
        chk(n_head == n_row, "header/row arity");
        chk(GaitAux::header()[0] == ',', "header 는 선행 콤마로 시작해야 한다");
        chk(render(g)[0] == ',',         "row 는 선행 콤마로 시작해야 한다");
    }

    // ── ② probe() 가 «update() 가 실제로 쓴 값» 을 나르는가 (LUT 분기) ──────
    // 🔴 재계산이 아니라 기록이어야 한다. 재계산하면 update() 로직이 바뀔 때 조용히 갈린다
    //    (실제로 정착(settle) 도입으로 표 조회 속도가 eff -> eff_g 로 바뀐 적이 있다).
    {
        MaskedLocoController c;
        auto& mg = c.mode_gait[1];
        mg.lut = true; mg.table = 2; mg.cadence = 1.15f; mg.turn_asym = true;
        mg.stance_z = MaskedLocoController::table_of(mg).stance_z;
        for (int i = 0; i < 40; ++i) c.update({0.4f, 0.0f, 0.0f}, 1);

        const GaitAux g = c.probe(1);
        chkf(g.phase,     c.phase_f,        "probe.phase == phase_f");
        chkf(g.stride_hz, c.last_stride_hz, "probe.stride_hz == update() 가 쓴 값");
        chkf(g.eff,       c.last_eff,       "probe.eff == update() 가 표에 넣은 값");
        chkf(g.foot_z_l,  c.foot_z[0],      "probe.foot_z_l");
        chkf(g.foot_z_r,  c.foot_z[1],      "probe.foot_z_r");
        chkf(g.bv_x,      c.base_vel[0],    "probe.bv_x");
        chk (g.lut == 1,                    "probe.lut == 1 (LUT 분기)");
        chk (g.cmd_mode == 1,               "probe.cmd_mode");

        // 위상이 stride_hz/50 만큼 나아가는가 = 기록값이 «시계를 실제로 돌린 그 값» 인가
        const float before = c.phase_f;
        c.update({0.4f, 0.0f, 0.0f}, 1);
        float step = c.phase_f - before; if (step < 0.0f) step += 1.0f;   // wrap
        chkf(step, c.last_stride_hz / 50.0f, "phase 진행량 == last_stride_hz/50", 1e-5f);

        // 그리고 그 값이 표에서 나온 값인가 (배포 파리티: cadence 가 곱해져 있어야 한다)
        const float want = gl_stride_freq(MaskedLocoController::table_of(mg),
                                          c.last_eff, c.is_run) * mg.cadence;
        chkf(c.last_stride_hz, want, "last_stride_hz == gl_stride_freq(표, 쓴eff) * cadence");
    }

    // ── ②-b 정착(settle) 중 — 여기서만 «재계산 == 기록» 이 깨진다 ────────────
    // 🔴 이 케이스가 이 테스트의 존재 이유다. 정착 중에는 명령이 0 이라
    //    effective_speed(base_vel) == 0 인데, update() 는 settle_eff(0.15) 로 표를 조회한다.
    //    계측이 재계산하면 «명령 0 인데 발이 움직인다» 를 0 으로 찍어 진단을 망친다.
    {
        MaskedLocoController c;
        auto& mg = c.mode_gait[1];
        mg.lut = true; mg.table = 2; mg.cadence = 1.15f;
        mg.stance_z = MaskedLocoController::table_of(mg).stance_z;
        mg.stand_deadzone = 0.1f;
        mg.settle_steps = 40;
        for (int i = 0; i < 30; ++i) c.update({0.4f, 0.0f, 0.0f}, 1);   // 걷다가
        c.update({0.0f, 0.0f, 0.0f}, 1);                                // 명령 0 -> 정착 시작
        const GaitAux g = c.probe(1);
        chk (g.settling == 1,        "정착이 시작돼야 한다 (테스트 전제)");
        chkf(g.bv_x, 0.0f,           "정착 중 명령은 0");
        chkf(g.eff, c.settle_eff,    "정착 중 eff 는 settle_eff — 재계산하면 0 이 나온다");
        chk (g.eff > 0.0f,           "정착 중 eff 가 0 이면 재계산하고 있는 것이다");
        chk (g.stride_hz > 0.0f,     "정착 중에도 시계는 돈다");
        chkf(g.stride_hz, c.last_stride_hz, "정착 중 stride_hz == update() 가 쓴 값");
    }

    // ── ③ quintic 분기도 채워지는가 (0 이 아니어야 한다) ────────────────────
    {
        MaskedLocoController c;
        c.mode_gait[3].lut = false;
        for (int i = 0; i < 10; ++i) c.update({0.3f, 0.0f, 0.0f}, 3);
        const GaitAux g = c.probe(3);
        chk (g.lut == 0,                                  "quintic 분기 lut == 0");
        chkf(g.stride_hz, 50.0f / float(c.period_steps),  "quintic stride_hz = 50/period_steps");
        chk (g.phase >= 0.0f && g.phase < 1.0f,           "quintic phase 는 [0,1)");
    }

    std::printf(fail ? "test_state_dump: %d FAIL\n" : "test_state_dump: OK\n", fail);
    return fail ? 1 : 0;
}

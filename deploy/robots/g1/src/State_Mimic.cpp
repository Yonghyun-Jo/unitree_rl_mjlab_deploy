#include "State_Mimic.h"
#include "RtPriority.h"
#include "LoopDiag.h"
#include "unitree_articulation.h"
#include "MaskedLocoController.h"   // deploy-clean foot_z gen + base_vel spline + arm-blend
#include "ModeRuntime.h"            // «지금 어느 모드인가» + 요청의 허용/거부 (ModeTable 의 성질)
#include "DeployFeatures.h"         // 슬롯이 요구하는 C++ 기능 ↔ 이 바이너리가 아는 기능
#include "HeightEstimator.h"
#include "Mode5Driver.h"
#include "MotionPreview.h"
#include "SafetyPolicy.h"           // 넘어짐·qd_warn 을 표의 안전 등급 + 명령·자세로 가른다
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <atomic>
#include <cstdlib>     // getenv (G1_POLICY_SLOT)
#include <cstring>
#include <thread>
#include <algorithm>   // std::clamp
#include <array>
#include <optional>    // g_commanded_upright → SafetyPolicy.h commanded_upright
#include <string>

extern std::string g_network_iface;   // main.cpp — 진단 부하 인터록 전용 (조작 경로 아님)
#include <string>
#include <cstdint>     // GUI shared-memory struct
#include <cstdio>      // printf (base_vel command readout)
#include <cstdlib>     // std::getenv (G1_FOOTZ_SRC 진단 스위치)

static Eigen::Quaternionf init_quat;
std::shared_ptr<State_Mimic::MotionLoader_> State_Mimic::motion = nullptr;
std::shared_ptr<State_Mimic::MotionLoader_> State_Mimic::motion_light = nullptr;
std::shared_ptr<State_Mimic::MotionLoader_> State_Mimic::motion_demo6 = nullptr;

// ===== 모드 =====
// 모드의 성질은 ModeTable.h(생성: mode_spec.py + config/modes.yaml)가 갖는다. 이 파일은 번호를 비교하지 않고
// g_mode.row() 의 성질을 묻는다 — 모드를 더하는 절차는 rules/ADDING_A_MODE.md.
static g1::ModeRuntime g_mode;
// ── 계약 v2 의 «이 틱의 값». 정책 루프가 env->step() 직전에 한 번 채우고, 관측 항은 읽기만 한다
//    (관측 항은 항마다 따로 불리므로 거기서 계산하면 미리보기 736칸을 세 번 만든다).
static float g_z_fk = 1e9f;                                  // 골반 높이 추정 (HeightEstimator.h)
static g1::TiltFilter g_tilt;                                // 걸러진 기울기 [deg]
static g1::safety::RecentHigh g_recent_high;                 // 최근 1 s 안에 «섰음»(명령 직립 ∧ z ≥ 0.65 ∧ 원시·걸러진 기울기 < 57.3°) (넘어짐 관문)
static g1::Mode5Driver g_m5;                                 // mode5 자세 버튼 상태
static g1::Mode5Driver::Cmd g_m5_cmd{};                      // mode5_cmd_live 가 아니면 전부 0
static std::vector<float> g_motion_block(g1::preview::DIM, 0.f);   // motion_preview 가 아니면 전부 0
// 지금 들어간 슬롯이 미리보기 관측 항(motion_preview_on/_k1/motion_preview)을 «선언» 했나.
// 생성자가 State_Mimic::obs_has_preview_ 에 담고 enter() 가 건다(slot_modes_ 와 같은 이유 — 인스턴스가 둘).
// 🔴 false(계약 v1 슬롯)면 미리보기를 계산하지 않고 클립 속도 배열도 요구하지 않는다: v1 슬롯도
//    mode4 를 알지만 그 ONNX 는 미리보기 입력이 없다 — 계산하면 없는 배열을 읽고, 요구하면 기동이 막힌다.
static bool g_obs_has_preview = false;
// 모드 이탈 조건이 보는 로봇 상태 (ModeRuntime::request). B1 에선 기본값(«직립으로 친다») 이었다.
static g1::ExitContext g_exit_ctx() { return g1::ExitContext{g_z_fk, g_tilt.value(), g_m5.standing_hold()}; }
// «직립» 은 한 가지 뜻이다 — 이탈 조건(ModeRuntime.h)과 안전 규칙(SafetyPolicy.h)이 같은 문턱을 쓴다.
static_assert(g1::safety::UPRIGHT_MIN_Z == g1::EXIT_MIN_Z, "SafetyPolicy.h UPRIGHT_MIN_Z != ModeRuntime.h EXIT_MIN_Z");
static_assert(g1::safety::QD_FALLBACK_MAX_TILT == g1::EXIT_MAX_TILT_DEG,
              "SafetyPolicy.h QD_FALLBACK_MAX_TILT != ModeRuntime.h EXIT_MAX_TILT_DEG");
// mode5 자세 키 안내 = 표(Mode5Presets.h ← config/mode5_keys.yaml)의 키, 표 순서. 손으로 쓰지 않는다.
// 정적 버퍼(힙 없음)를 처음 부를 때 한 번 채운다 — 부르는 곳은 정책 스레드 하나(전환 에지).
static const char* g_m5_key_hint() {
    static char buf[2 * m5::N_PRESETS + 1] = {};
    if (!buf[0]) {
        int n = 0;
        for (const m5::Preset& p : m5::PRESETS)
            if (p.key != '\0') { if (n) buf[n++] = ' '; buf[n++] = p.key; }
    }
    return buf;
}
static constexpr int G1_N_LOWER = 12;
// 안전 폴백이 돌아갈 모드 = 표의 첫 행(명령만으로 서서 걷는 모드, 하드웨어에서 완전 관측 가능).
// ModeTable 의 «범위 밖 요청 -> 첫 행» 과 같은 약속이다 — 번호를 여기 한 곳에서만 이름으로 받는다.
static constexpr int G1_FALLBACK_MODE = mode_table::ROWS[0].id;

// 클립 슬롯 — ref_source=clip 인 모드가 읽는 클립들. 0 = 주 클립(motion_file), 그 뒤 = 설정에 있으면.
// rewind_on_enter: 그 모드에 «들어올 때마다» 처음부터 다시 튼다(옛 demo6 거동). 아니면 episode 시계 그대로(나갔다 오면 이어서).
struct ClipSlot {
    std::shared_ptr<State_Mimic::MotionLoader_> loader;
    const char* name;
    bool rewind_on_enter;
    float t0;                       // rewind_on_enter 클립의 시계 원점 [s]
};
static std::vector<ClipSlot> g_clips;
static void g_build_clips() {
    g_clips.clear();
    if (State_Mimic::motion)       g_clips.push_back({State_Mimic::motion,       "primary", false, 0.f});
    if (State_Mimic::motion_light) g_clips.push_back({State_Mimic::motion_light, "light",   false, 0.f});
    if (State_Mimic::motion_demo6) g_clips.push_back({State_Mimic::motion_demo6, "demo6",   true,  0.f});
    // 슬롯이 줄어들 수 있다(State_Mimic 은 두 번 만들어지고 enter() 마다 주 클립이 다시 묶인다)
    // -> 고른 칸이 사라졌으면 주 클립으로 되돌린다. 범위 안이면 고른 그대로 둔다.
    if (g_mode.clip_id() >= (int)g_clips.size()) g_mode.select_clip(0, (int)g_clips.size());
}
static inline ClipSlot* active_clip() {
    const int i = g_mode.clip_id();
    return (i >= 0 && i < (int)g_clips.size()) ? &g_clips[i] : nullptr;
}
// 참조를 읽을 로더: 클립 모드면 선택된 클립, 아니면 주 로더(VR 버퍼를 갖고 있다).
static inline std::shared_ptr<State_Mimic::MotionLoader_> active_ref_loader() {
    if (g_mode.row().ref_source == mode_table::RefSource::Clip)
        if (ClipSlot* c = active_clip()) return c->loader;
    return State_Mimic::motion;
}
static inline bool g_ref_is_clip() { return g_mode.row().ref_source == mode_table::RefSource::Clip; }
// 넘어짐 관문(SafetyPolicy.h commanded_upright)에 «지금 명령의 높이» 를 모아 넘긴다 — 표의 성질로만(모드 번호 없음).
//   ref_source=clip → 재생 중인 클립의 현재 프레임 골반 높이. z() 는 root_positions(모든 클립에 있다)만
//                     읽는다 — lin()/ang() 는 부르지 않는다(속도 배열은 미리보기 슬롯의 클립에만 있다).
//   mode5_cmd_live  → 활성 자세의 목표 높이(Mode5Presets.h).
//   그 밖           → 명령에 높이가 없다 → «직립 아님».
// 정책 스레드 전용: g_mode · g_clips · 로더 frame · g_m5 가 전부 그 스레드의 것이다.
static bool g_commanded_upright() {
    const mode_table::Row& row = g_mode.row();
    std::optional<float> clip_z;
    if (row.ref_source == mode_table::RefSource::Clip)
        if (ClipSlot* c = active_clip())
            if (c->loader && c->loader->frame >= 0 && c->loader->frame < (int)c->loader->root_positions.size())
                clip_z = c->loader->z(c->loader->frame);
    std::optional<double> m5_z;
    if (row.mode5_cmd_live && g_m5.active()) m5_z = m5::PRESETS[g_m5.preset()].z;
    return g1::safety::commanded_upright(row, clip_z, m5_z);
}

// Deploy-clean controller (1:1 with mjlab_g1_motion loco_controller.py; golden-verified).
// Updated once per policy step (see policy_thread) BEFORE obs are computed; obs terms +
// run() read its cached base_vel / foot_z / arm_scale. ⚠ params assume 50 Hz control.
static MaskedLocoController g_loco;

// ── 🔬 진단 A/B: foot_z==Ref 인 모드의 발-z 원천 (env `G1_FOOTZ_SRC`) ──────────────
//   ref  (기본) = 레퍼런스 발 world-z. 학습 원장과 같다(aedcc77).
//   gen         = 생성기. aedcc77 이전의 종전 배포 동작 (그 모드들은 base_vel 이 0 이라 stance 상수).
//   ramp        = ref 이되 «스위치 램프» 를 같이 탄다 — 다리 q_ref 는 switch_alpha 로 1초에 걸쳐
//                 현재자세→클립으로 램프되는데(masked_joint_command), 발-z 만 즉시 점프하면
//                 그 1초 동안 «다리는 아직 서 있는데 발은 0.65 m» 라는 모순된 짝을 먹인다.
// 왜 스위치로 두나: sim 한 번 돌려 «어느 쪽이 원인인지» 를 재빌드 없이 가르기 위해서다.
enum class FootZSrc { Ref, Gen, Ramp };
static FootZSrc g_footz_src = FootZSrc::Ref;
static const char* g_footz_src_name = "ref";
static void g_load_footz_src() {
    const char* e = std::getenv("G1_FOOTZ_SRC");
    if (e && *e) {
        std::string v(e);
        if      (v == "gen")  { g_footz_src = FootZSrc::Gen;  g_footz_src_name = "gen"; }
        else if (v == "ramp") { g_footz_src = FootZSrc::Ramp; g_footz_src_name = "ramp"; }
        else if (v == "ref")  { g_footz_src = FootZSrc::Ref;  g_footz_src_name = "ref"; }
        else spdlog::warn("[diag] G1_FOOTZ_SRC='{}' 는 모르는 값 — ref 유지", v);
    }
    spdlog::info("[diag] ref_foot_height 원천 = {} (표의 foot_z==Ref 인 모드에만 영향)", g_footz_src_name);
}

// Accumulated keyboard velocity command (walker_teleop.py style: each keypress ±STEP, space=reset).
// Coexists with the joystick stick (base_vel_command sums them). Edge-triggered so one tap = one step.
static float g_kb_vx = 0.0f, g_kb_vy = 0.0f, g_kb_wz = 0.0f;
static std::string g_kb_last = "";
// deploy base_vel caps — set to the TRAINING base_vel range (20-motion manifest, yaw-local):
//   vx p99=3.10 (max 5.4) → 3.0 (running OK);  vy |.|p99=1.88 (lateral sparse) → 1.5;
//   wz |.|p99=4.88 → 2.0 (turning).
// 🔴 base_vel 하드캡 = «학습 봉투» 다. 임의로 키우면 그만큼 OOD 로 나간다.
//   출처: mjlab_g1_motion/tasks/stage4_mode1_env_cfg.py:39
//     CMD_BASE_VEL = vx(-1.5, 2.5) · vy(-0.8, 0.8) · wz(-2.0, 2.0)
//   ⚠ vx 는 «비대칭» 이다 — 전진 2.5 / 후진 1.5. 대칭 클램프로 두면 후진이 학습의 1.67배로 나간다.
//   종전 값(3.0 / 1.5 / 2.0)의 근거는 «20-motion manifest 클립 속도 p99» 였는데, 지금 mode1 은
//   클립이 아니라 CMD_BASE_VEL 로 학습한다 = 낡은 근거였다. 후진 2.0배·횡 1.9배 OOD 였다.
//   ⚠ mode2 봉투는 더 좁다(stage4_mode2_env_cfg.py:41 — vx(-1.0, 1.5)). 여기 값은 mode1 기준이라
//     mode2 에선 여전히 전진 1.67배가 가능하다. 모드별 봉투 분리는 별건으로 남겨 둔다.
static constexpr float KB_STEP     = 0.1f;
static constexpr float VX_MAX_FWD  = 2.5f;   // 전진 상한
static constexpr float VX_MAX_BWD  = 1.5f;   // 후진 상한(크기)
static constexpr float KB_MAXVY    = 0.8f;
static constexpr float KB_MAXW     = 2.0f;
// vx 전용 비대칭 클램프. 이 함수 밖에서 vx 를 clamp 하지 말 것(대칭으로 새기 쉽다).
static inline float clamp_vx(float v) { return std::clamp(v, -VX_MAX_BWD, VX_MAX_FWD); }

// ── Optional browser-GUI control channel (mjlab-style viser GUI -> shared memory) ──
// A Python viser GUI (deploy/robots/g1/tools/masked_gui.py) writes this packed struct to
// /dev/shm/g1_masked_gui; g_poll_gui() reads it each step and overrides mode / base_vel /
// foot-gen params. Optional: if the file is absent, keyboard + joystick drive everything.
#pragma pack(push, 1)
struct GuiCtrl {                 // v2 (2026-09-22). tools/gui_shm.py FMT 와 같은 커밋으로만 바꾼다
    int32_t  magic;              // GUI_CTRL_MAGIC
    uint32_t seq;                // GUI 가 바꿀 때마다 +1 (에지 적용)
    int32_t  mode_req;           // 0 = 요청 없음(1회성). 1.. = 표의 모드 (슬롯 지원·이탈 조건은 ModeRuntime)
    float    vx, vy, wz;         // base_vel 명령 (배포 상한은 아래에서 한 번 더)
    int32_t  period_steps;       // foot-gen gait period
    float    height_scale;       // foot-gen swing-height multiplier
    float    turn_k;             // foot-gen |wz|->step gain
    int32_t  m5_preset;          // 0 = 없음, 1..N = Mode5Presets.h PRESETS[i-1]
    uint32_t m5_press_seq;       // 누를 때마다 +1 (같은 자세 재입력 = 새 목표)
    int32_t  clip_req;           // −1 = 그대로, 0.. = 고를 클립 (재생 중이면 거부). 1회성
};
#pragma pack(pop)
static_assert(sizeof(GuiCtrl) == 12 * 4, "GuiCtrl = gui_shm.FMT 12칸 48 바이트 (tests/test_gui_shm_layout.py)");
static constexpr int32_t GUI_CTRL_MAGIC = 0x6704;      // 0x6701 gui v1 · 0x6702 vr · 0x6703 estop 과 겹치지 않게
static constexpr int32_t GUI_CTRL_MAGIC_V1 = 0x6701;   // 옛 형식 — 읽지 않는다(한 번 알린다)
static uint32_t g_gui_last_seq = 0;
static uint32_t g_gui_last_m5_seq = 0;                   // 자세가 실린 프레임만 옮긴다(PICO·replay 는 자세 칸이 0)
// 체류(Mimic 진입)마다 한 번 «기준선 다시 잡기». enter() 가 세우고, 그 체류의 첫 g_poll_gui 가 내린다.
//   파일이 있으면(유효한 v2) seq·자세 seq 를 기준으로만 잡고 **아무것도 적용하지 않는다** — Mimic 밖
//   (Passive·FixStand·지난 세션)에서 눌린 모드·자세·클립·속도가 p→f→m 뒤 첫 틱에 발동하지 않게.
//   파일이 없으면 플래그만 내린다 → 그 체류의 첫 실제 쓰기는 그대로 먹는다.
static bool g_gui_rebase = true;

// 조작자의 모드 요청이 들어오는 유일한 문. 거부되면 이유를 한 줄 남긴다(같은 모드 재요청은 조용히 통과).
// ⚠ GUI·VR 은 50 Hz 로 들어온다 — 같은 거부를 매 틱 찍으면 정책 스레드에서 초당 50번 I/O 다.
//    그래서 «값이 바뀔 때만» 남긴다.
// 🔴 «아무것도 안 눌렀다» 표시는 -1 이다. 0 이 아니다 — 초기화 안 된 shm 구조체는 0 으로 읽히므로
//    0 을 센티넬로 쓰면 그 쓰레기 값의 거부가 영영 안 찍힌다(있는데 안 보이는 고장).
static constexpr int G1_REJECT_NONE = -1;
static int g_reject_last = G1_REJECT_NONE;
static void g_request_mode(int m, const char* src) {
    const g1::ModeResult r = g_mode.request(m, g_exit_ctx());
    if (r.accepted) { g_reject_last = G1_REJECT_NONE; return; }
    if (m == g_reject_last) return;
    g_reject_last = m;
    printf("\r\n[cmd_mode] %s -> %d 거부: %s\r\n", src, m, r.reason); fflush(stdout);
}

// 채널별로 요청할 수 있는 모드 (옛 «1 <= cmd_mode <= 3» 범위 검사를 성질로 옮긴 것을 채널마다 가른다).
// GUI = 사람이 화면을 보고 누르는 버튼(v2 부터 모드 4·5 버튼) → 표의 모든 모드. 슬롯이 아는가·지금 나갈 수
//       있는가는 ModeRuntime 이 본다(이탈 조건에 실측 z_fk·기울기).
// VR  = 남의 프로그램이 50 Hz 로 쓰는 바이트 → 직립 모드만(옛 «1 <= cmd_mode <= 3» 의 성질판, 그대로).
//       쓰레기 값이 저자세·클립재생 모드를 켜지 못하게 하는 필터이기도 하다.
enum class Channel { Gui, Vr };
static inline bool g_channel_may_request(int m, Channel ch) {
    if (!mode_table::valid(m)) return false;
    switch (ch) {
        case Channel::Gui: return true;
        case Channel::Vr:  return mode_table::row(m).safety == mode_table::Safety::UprightOnly;
    }
    return false;
}
// 채널에서 들어온 요청. 자격 없는 값은 옛 코드처럼 «아무 일도 안 일어난다»(한 줄만 남긴다).
static void g_request_mode_from_channel(int m, Channel ch, const char* src) {
    if (g_channel_may_request(m, ch)) { g_request_mode(m, src); return; }
    if (m == g_reject_last) return;                      // 50 Hz — 같은 값은 한 번만
    g_reject_last = m;
    printf("\r\n[cmd_mode] %s -> %d 무시: %s\r\n", src, m,
           ch == Channel::Vr ? "이 채널은 직립 모드만 요청한다" : "표에 없는 모드"); fflush(stdout);
}

// 클립 선택의 유일한 문(키보드 [ ] · GUI). 클립 모드가 «들어가기 전에» 고른다.
// 🔴 재생 중(ref_source=clip)에는 바꾸지 않는다. 클립 정체가 바뀌면 참조가 통째로 갈리는데,
//    그것을 무해하게 만드는 넷(crossfade · 다리 q_ref 램프 · 되감기 · init_quat 재앵커)은
//    전부 «전환 블록»(g_mode.consume_switch()) 안에서만 돈다. 여기서 바꾸면 그 밖이다:
//    masked_joint_command 가 switch_alpha==1.0 으로 새 로더를 즉시 읽어 전신 q_ref 가
//    한 틱에 점프하고, init_quat 은 옛 클립에 앵커된 채 남고, 되감는 칸은 t0 가 낡아
//    중간 프레임부터 시작하며, ref_foot_height 도 같이 튄다. 실기에선 모터 보호정지다.
//    옛 코드에는 이 구멍이 없었다 — 클립 정체는 «모드 전환과 함께만» 바뀌었다.
static void g_select_clip(int id, const char* src) {
    if (g_ref_is_clip()) {
        printf("\r\n[clip] %s: 재생 중에는 못 바꾼다 — 먼저 다른 모드로 나갔다가 고를 것\r\n", src); fflush(stdout);
        return;
    }
    const int n = (int)g_clips.size();
    if (n == 0) { printf("\r\n[clip] %s: 이 슬롯엔 클립이 없다\r\n", src); fflush(stdout); return; }
    if (!g_mode.select_clip(id, n)) { printf("\r\n[clip] %s: 없는 칸 %d (0..%d)\r\n", src, id, n - 1); fflush(stdout); return; }
    printf("\r\n[clip] %d/%d «%s»\r\n", id, n, g_clips[id].name); fflush(stdout);
}

static void g_poll_gui()
{
    FILE* f = std::fopen("/dev/shm/g1_masked_gui", "rb");
    if (!f) { g_gui_rebase = false; return; }                         // 기준선 없음 → 첫 실제 쓰기가 먹는다
    GuiCtrl g{};
    const size_t n = std::fread(&g, 1, sizeof(g), f);
    std::fclose(f);
    if (n >= sizeof(int32_t) && g.magic == GUI_CTRL_MAGIC_V1) {
        static bool warned = false;
        if (!warned) { warned = true; spdlog::warn("[gui] 옛 형식(0x6701) — 무시한다. tools/gui_shm.py 를 이 브랜치 것으로 (GUI·PICO 브리지 재시작)"); }
        return;
    }
    if (n != sizeof(g) || g.magic != GUI_CTRL_MAGIC) return;         // (기준선은 유효한 v2 프레임을 볼 때까지 미룬다)
    if (g_gui_rebase) {
        // 1회성 요청도 파일에는 «마지막 쓰기» 로 남는다 — 그대로 먹으면 Mimic 밖에서 누른 «5» 가 m 직후
        // 재생된다(서 있던 로봇이 드러눕는다). 속도·foot-gen 도 적용하지 않는다: 체류마다 다시 잡으므로
        // 적용하면 키보드로 멈춘(space) 뒤 p→f→m 할 때 GUI 에 남은 옛 속도로 걷기 시작한다.
        g_gui_rebase = false;
        g_gui_last_seq = g.seq;
        if (g.m5_preset != 0) g_gui_last_m5_seq = g.m5_press_seq;    // 자세 없는 프레임(PICO·replay)은 기준을 안 옮긴다
        printf("\r\n[gui] 진입 기준선 seq=%u — 파일에 남은 요청(mode_req=%d 자세=%d clip_req=%d)·속도는 적용하지 않는다. 다음 쓰기부터 먹는다\r\n",
               g.seq, g.mode_req, g.m5_preset, g.clip_req);
        fflush(stdout);
        return;
    }
    if (g.seq == g_gui_last_seq) return;
    g_gui_last_seq = g.seq;
    if (g.mode_req != 0) {
        g_reject_last = G1_REJECT_NONE;   // v2 의 mode_req 는 사람이 누른 1회성 입력 → 거부도 매번 한 줄 (키보드와 같다)
        g_request_mode_from_channel(g.mode_req, Channel::Gui, "gui");
    }
    // 자세가 실린 프레임만 누름으로 본다 — 자세 칸이 0 인 쓰는 쪽(PICO 브리지·replay_cmd)의 프레임이
    // 기준을 0 으로 옮기면, 다음 GUI 프레임이 GUI 의 마지막 자세(예: 드러누움)를 «다시 누름» 으로 만든다.
    if (g.m5_preset != 0 && g.m5_press_seq != g_gui_last_m5_seq) {
        g_gui_last_m5_seq = g.m5_press_seq;
        const int p = g.m5_preset - 1;
        if (p < 0 || p >= m5::N_PRESETS) {
            printf("\r\n[m5] gui: 없는 자세 %d\r\n", g.m5_preset);
        } else if (!g_mode.row().mode5_cmd_live) {
            printf("\r\n[m5] gui «%s» — mode5 에서만 (지금 mode%d %s)\r\n", m5::PRESETS[p].name, g_mode.row().id, g_mode.row().name);
        } else {
            g_m5.press(p);
            printf("\r\n[m5] gui → «%s»\r\n", m5::PRESETS[p].name);
        }
        fflush(stdout);
    }
    if (g.clip_req >= 0) g_select_clip(g.clip_req, "gui");
    g_kb_vx = g.vx; g_kb_vy = g.vy; g_kb_wz = g.wz;                   // clamped in g_joystick_base_vel
    if (g.period_steps > 0)  g_loco.period_steps = g.period_steps;
    if (g.height_scale > 0)  g_loco.height_scale = g.height_scale;
    g_loco.turn_k = g.turn_k;
    // verify commands land (printed once per GUI change, not per step): if mode flaps or
    // base_vel oscillates here, the GUI is fighting the keyboard/joystick.
    printf("\r\n[gui seq=%u] mode=%d base_vel=[%+.2f %+.2f %+.2f] foot{period=%d hscale=%.2f turnk=%.2f}\r\n",
           g.seq, g_mode.mode(), g_kb_vx, g_kb_vy, g_kb_wz,
           g_loco.period_steps, g_loco.height_scale, g_loco.turn_k);
    fflush(stdout);
}

// ── VR teleop reference channel (variant B) ──────────────────────────────────────
// A com1-local bridge (teleop/zmq_to_vr_bridge.py from ZMQ+GMR, or teleop/vr_replay.py for
// a recorded clip) writes this to /dev/shm/g1_vr_ref. g_poll_vr() feeds it into the
// MotionLoader so the masked obs (masked_joint_command / masked_root_ori_b) read the VR
// reference instead of the clip. base_vel/mode come from the VR thumbstick/buttons.
// valid=0 -> clear override (back to clip). ⚠ root_quat is wxyz (= GMR qpos[3:7]).
#pragma pack(push, 1)
struct VrRef {
    int32_t  magic;        // 0x6702
    uint32_t seq;
    int32_t  valid;
    int32_t  cmd_mode;     // 2 or 3
    float    base_vel[3];  // [vx,vy,wz] yaw-local thumbstick
    float    root_quat[4]; // pelvis wxyz
    float    dof_pos[29];  // = GMR qpos[7:36]
    float    dof_vel[29];  // finite-diff + EMA
    // ── v2 확장 (2026-08-20). 레퍼런스 발 world-z [z_L, z_R].
    // mode3 은 학습에 FOOT_GEN 이 없어 «레퍼런스 발 z» 가 obs 원천이다(생성기 아님).
    // ⚠ 구버전 publisher 는 여기까지 안 쓴다 → 아래 g_poll_vr 이 «짧은 파일»도 받아들이고,
    //   그 경우 이 필드는 0 으로 남아 사용되지 않는다(obs 는 stance 로 폴백).
    float    foot_z[2];
};
#pragma pack(pop)
// 확장 전 레이아웃 크기 — 이 길이로 온 파일도 정상 수신한다(구버전 호환).
static constexpr size_t VR_REF_LEGACY_BYTES = sizeof(VrRef) - sizeof(float) * 2;
static uint32_t g_vr_last_seq = 0;
static int g_vr_stale = 0;                      // polls since seq last advanced
static constexpr int VR_STALE_MAX = 25;         // ~0.5s @50Hz: writer stalled/dead -> release to clip

static void g_poll_vr()
{
    FILE* f = std::fopen("/dev/shm/g1_vr_ref", "rb");
    if (!f) {   // no VR channel -> release any active override (back to clip)
        if (State_Mimic::motion && State_Mimic::motion->vr_override) State_Mimic::motion->clear_vr();
        return;
    }
    VrRef v{};
    unsigned char buf[sizeof(VrRef)] = {};
    size_t n = std::fread(buf, 1, sizeof(buf), f);
    std::fclose(f);
    // 신(발-z 포함) / 구 레이아웃 둘 다 수신. v 는 0 초기화라 구버전이면 foot_z 가 0 으로 남는다.
    if (n != sizeof(VrRef) && n != VR_REF_LEGACY_BYTES) return;
    std::memcpy(&v, buf, n);
    const bool vr_has_foot_z = (n == sizeof(VrRef));
    if (v.magic != 0x6702) return;
    if (v.seq == g_vr_last_seq) {   // LIVENESS: seq frozen -> writer stalled/dead (or stale zombie)
        if (State_Mimic::motion && State_Mimic::motion->vr_override && ++g_vr_stale > VR_STALE_MAX)
            State_Mimic::motion->clear_vr();     // auto-return to clip (VR 끊김 안전, c00ac28 의도)
        return;
    }
    g_vr_last_seq = v.seq;
    g_vr_stale = 0;
    if (!v.valid) { if (State_Mimic::motion) State_Mimic::motion->clear_vr(); return; }
    g_request_mode_from_channel(v.cmd_mode, Channel::Vr, "vr");
    g_kb_vx = v.base_vel[0]; g_kb_vy = v.base_vel[1]; g_kb_wz = v.base_vel[2];
    if (State_Mimic::motion) {
        Eigen::VectorXf dp = Eigen::VectorXf::Map(v.dof_pos, 29);
        Eigen::VectorXf dv = Eigen::VectorXf::Map(v.dof_vel, 29);
        Eigen::Quaternionf rq(v.root_quat[0], v.root_quat[1], v.root_quat[2], v.root_quat[3]);  // wxyz
        State_Mimic::motion->set_vr(dp, dv, rq, {v.foot_z[0], v.foot_z[1]}, vr_has_foot_z);
    }
}

// Poll joystick d-pad (직립 모드 3개) + keyboard (표의 모드 키 전부) and keyboard velocity accumulation.
// Called every policy step (same thread as obs). Both input modes are always live.
static void g_poll_inputs(isaaclab::ManagerBasedRLEnv* env)
{
    g_poll_gui();   // browser GUI (shared memory) overrides; no-op if the file is absent
    // --- joystick d-pad -> mode (avoids A/B used by FSM transitions) ---
    if (auto joy = env->robot->data.joystick) {
        if      (joy->left.on_pressed)  g_request_mode(1, "joy");
        else if (joy->up.on_pressed)    g_request_mode(2, "joy");
        else if (joy->right.on_pressed) g_request_mode(3, "joy");
    }
    // --- keyboard (edge-triggered: act once per key change) ---
    if (!FSMState::keyboard) return;
    std::string k = FSMState::keyboard->key();
    if (k == g_kb_last) return;
    g_kb_last = k;
    if (k.empty()) return;
    g_reject_last = G1_REJECT_NONE;   // 사람이 «누른» 입력에는 매번 한 줄 답한다 (중복 억제는 50 Hz 채널 전용)
    bool vel_changed = false;
    if      (k == "w") { g_kb_vx = clamp_vx(g_kb_vx + KB_STEP); vel_changed = true; }  // forward
    else if (k == "s") { g_kb_vx = clamp_vx(g_kb_vx - KB_STEP); vel_changed = true; }  // backward
    else if (k == "a") { g_kb_vy = std::clamp(g_kb_vy + KB_STEP, -KB_MAXVY, KB_MAXVY); vel_changed = true; }  // strafe left
    else if (k == "d") { g_kb_vy = std::clamp(g_kb_vy - KB_STEP, -KB_MAXVY, KB_MAXVY); vel_changed = true; }  // strafe right
    else if (k == "q") { g_kb_wz = std::clamp(g_kb_wz + KB_STEP, -KB_MAXW, KB_MAXW); vel_changed = true; }  // yaw CCW (반시계)
    else if (k == "e") { g_kb_wz = std::clamp(g_kb_wz - KB_STEP, -KB_MAXW, KB_MAXW); vel_changed = true; }  // yaw CW  (시계)
    else if (k == " ") { g_kb_vx = g_kb_vy = g_kb_wz = 0.0f;                          vel_changed = true; }  // stop
    else if (k == "[" || k == "]") {                       // 클립 선택 — 재생 중 거부는 g_select_clip 안에서
        const int n = (int)g_clips.size();
        if (n > 0) g_select_clip((g_mode.clip_id() + (k == "]" ? 1 : n - 1)) % n, "key");
    }
    else if (k.size() == 1 && m5::preset_by_key(k[0]) >= 0) {   // mode5 자세 버튼 (config/mode5_keys.yaml)
        const int p = m5::preset_by_key(k[0]);
        if (!g_mode.row().mode5_cmd_live) {
            printf("\r\n[m5] «%s» — mode5 에서만 (지금 mode%d %s)\r\n", m5::PRESETS[p].name,
                   g_mode.row().id, g_mode.row().name);
        } else {
            g_m5.press(p);
            printf("\r\n[m5] → «%s» (목표 높이 %.2f m)\r\n", m5::PRESETS[p].name, m5::PRESETS[p].z);
        }
        fflush(stdout);
    }
    else {
        for (int m = 1; m <= mode_table::N_MODES; ++m) {
            const mode_table::Row& M = mode_table::row(m);
            if (k.size() == 1 && k[0] == M.key) {
                g_request_mode(m, "key");
                if (g_mode.mode() == m) { printf("\r\n[cmd_mode] -> %d (%s)\r\n", m, M.name); fflush(stdout); }
                break;
            }
        }
    }

    // Print the base_vel command ONLY when a velocity key changed it (not every frame).
    // 명령을 안 쓰는 모드는 downstream 에서 base_vel 을 0 으로 눌러 버리므로 그 사실을 같이 보인다.
    if (vel_changed) {
        const bool active = g_mode.row().base_vel_live;
        printf("\r\n[base_vel cmd] vx=%+.2f  vy=%+.2f  wz=%+.2f   (mode %d%s)\r\n",
               g_kb_vx, g_kb_vy, g_kb_wz, g_mode.mode(), active ? "" : ", 이 모드는 base_vel 을 안 쓴다");
        fflush(stdout);
    }
}


// Raw joystick + keyboard base_vel target [vx,vy,wz] (UNmasked; the controller masks mode3).
// Fed to g_loco.update() each step; base_vel_command obs returns the controller's splined value.
// ⚠ 조이스틱 스케일과 키보드 캡은 «학습 봉투»(CMD_BASE_VEL)와 같아야 한다 — 위 상수 주석 참조.
static std::array<float, 3> g_joystick_base_vel(isaaclab::ManagerBasedRLEnv* env)
{
    float jx = 0.0f, jy = 0.0f, jw = 0.0f;
    if (auto joy = env->robot->data.joystick) {
        // full-stick = the deploy cap (training range), consistent with GUI/PICO.
        constexpr float DEAD = 0.08f;
        // deadzone: a real pad always exists (data.joystick != null); centered-stick drift
        // would otherwise ADD to the GUI/keyboard command and make the robot judder.
        auto dz = [](float v, float d) { return std::abs(v) < d ? 0.0f : v; };
        // 풀스틱 = 학습 봉투의 끝. vx 는 미는 방향에 따라 스케일이 다르다(전진 2.5 / 후진 1.5).
        const float sx = dz(joy->ly(), DEAD);
        jx =  sx * (sx >= 0.0f ? VX_MAX_FWD : VX_MAX_BWD);
        jy = -KB_MAXVY * dz(joy->lx(), DEAD);
        jw = -KB_MAXW  * dz(joy->rx(), DEAD);
    }
    return { clamp_vx(g_kb_vx + jx),
             std::clamp(g_kb_vy + jy, -KB_MAXVY, KB_MAXVY),
             std::clamp(g_kb_wz + jw, -KB_MAXW, KB_MAXW) };
}

Eigen::Quaternionf robot_quat_w(isaaclab::ManagerBasedRLEnv* env)
{
    using G1Type = unitree::BaseArticulation<LowState_t::SharedPtr>;
    G1Type* robot = dynamic_cast<G1Type*>(env->robot.get());

    auto root_quat = env->robot->data.root_quat_w;
    auto & motors = robot->lowstate->msg_.motor_state();

    Eigen::Quaternionf torso_quat = root_quat \
        * Eigen::AngleAxisf(motors[12].q(), Eigen::Vector3f::UnitZ()) \
        * Eigen::AngleAxisf(motors[13].q(), Eigen::Vector3f::UnitX()) \
        * Eigen::AngleAxisf(motors[14].q(), Eigen::Vector3f::UnitY());

//    return root_quat;
    return torso_quat;
}

Eigen::Quaternionf motion_anchor_quat_w(std::shared_ptr<State_Mimic::MotionLoader_> loader)
{
    const auto root_quat = loader->root_quaternion();
    const auto joint_pos = loader->joint_pos();
    Eigen::Quaternionf torso_quat = root_quat \
        * Eigen::AngleAxisf(joint_pos[12], Eigen::Vector3f::UnitZ()) \
        * Eigen::AngleAxisf(joint_pos[13], Eigen::Vector3f::UnitX()) \
        * Eigen::AngleAxisf(joint_pos[14], Eigen::Vector3f::UnitY());

//    return root_quat;
    return torso_quat;
}


namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(motion_command)
{
    auto loader = active_ref_loader();
    std::vector<float> data;

    auto motion_joint_pos = loader->joint_pos();
    auto motion_joint_vel = loader->joint_vel();

    data.insert(data.end(),
                motion_joint_pos.data(),
                motion_joint_pos.data() + motion_joint_pos.size());
    data.insert(data.end(),
                motion_joint_vel.data(),
                motion_joint_vel.data() + motion_joint_vel.size());
    return data;
}

REGISTER_OBSERVATION(motion_anchor_ori_b)
{
    auto loader = active_ref_loader();
    std::vector<float> out;

    auto real_quat_w = robot_quat_w(env);
    auto ref_quat_w  = motion_anchor_quat_w(loader);

    auto rot_ = (init_quat * ref_quat_w).conjugate() * real_quat_w;
    // ⚠ auto 금지: toRotationMatrix() 는 Matrix3f 를 값으로 반환 -> .transpose() 는 그 임시를
    //   참조만 하는 표현식이라 auto 로 받으면 임시 소멸 후 해제된 스택을 읽는다(UB).
    //   -O0 에선 우연히 맞고 -O2/-O3 에선 쓰레기. 실기(-O3)에서만 터진 원인.
    const Eigen::Matrix3f rot = rot_.toRotationMatrix().transpose();

    Eigen::Matrix<float, 6, 1> data;
    data << rot(0, 0), rot(0, 1), rot(1, 0), rot(1, 1), rot(2, 0), rot(2, 1);
    return std::vector<float>(data.data(), data.data() + data.size());
}

// ---- Masked-3mode obs (mirror mjlab_g1_motion g1_mimic_env.calc_masked_*) ----

// q_ref(29)+q̇_ref(29) = 58, lower[:12]/upper[12:] groups zeroed per mode (추종하는 쪽만 남는다).
REGISTER_OBSERVATION(masked_joint_command)
{
    auto loader = active_ref_loader();
    const bool clip = g_ref_is_clip();   // 클립 재생 모드 = 클립 버퍼; 그 외 = VR 버퍼(클립 아님)
    auto q  = clip ? loader->joint_pos_clip() : loader->joint_pos_vr();
    auto qd = clip ? loader->joint_vel_clip() : loader->joint_vel_vr();
    const bool mu = g_mode.row().track_upper, ml = g_mode.row().track_lower;
    // Lower-body reference smoothing (하체를 추종하는 모드): ramp the LEG q_ref from the robot's CURRENT
    // measured pose toward the clip/VR target over the switch window (reuse switch_alpha), so the
    // policy tracks a smoothly-moving leg target and produces NATURAL STEPPING instead of a
    // joint-space slide. lb==1.0 outside a switch -> no effect. Upper body uses apply_switch_blend.
    if (ml && g_loco.leg_fresh) {
        for (int i = 0; i < G1_N_LOWER; ++i) g_loco.leg_from[i] = env->robot->data.joint_pos[i];
        g_loco.leg_fresh = false;
    }
    const float lb = g_loco.switch_alpha;
    std::vector<float> data;
    data.reserve(q.size() + qd.size());
    for (int i = 0; i < q.size(); ++i) {                 // q_ref (position)
        float x = q[i];
        if (i < G1_N_LOWER) {                            // leg
            if (!ml) x = 0.0f;                           // generated -> 0
            else if (lb < 1.0f) x = (1.0f - lb) * g_loco.leg_from[i] + lb * x;  // ramp current->target
        } else if (!mu) {                                // upper generated -> 0
            x = 0.0f;
        }
        data.push_back(x);
    }
    for (int i = 0; i < qd.size(); ++i) {                // qd_ref (velocity feedforward)
        float x = qd[i];
        if (i < G1_N_LOWER) {
            if (!ml) x = 0.0f;
            else if (lb < 1.0f) x = lb * x;              // leg feedforward ramps in from 0
        } else if (!mu) {
            x = 0.0f;
        }
        data.push_back(x);
    }
    return data;
}

// 계약 v1 = 표의 앞 2칸([mask_upper, mask_lower]). 🔴 폭을 늘리면 지금 배포된 ONNX(입력 1640)가
// 어긋난다 — 8칸 전부를 내는 것은 계약 v2 의 일이다.
REGISTER_OBSERVATION(command_mask)
{
    const auto& b = g_mode.row().bits;
    return std::vector<float>{ b[0], b[1] };
}

// ── 계약 v2 관측 항 (spec §4.2). 계약 v1 슬롯은 이 항들을 선언하지 않으므로 거동이 안 바뀐다.
// 모드 마스크 8칸 = 표의 비트 전부 (학습 `mask`, mode_spec.MASK_DIM). v1 은 위 command_mask(앞 2칸).
REGISTER_OBSERVATION(mode_mask)
{
    const auto& b = g_mode.row().bits;
    return std::vector<float>(b.begin(), b.end());
}
// mode5 명령 53칸 (학습 `mode5_cmd`). mode5_cmd_live 가 아닌 모드 = 정확히 0 (정책 루프가 채운다).
REGISTER_OBSERVATION(mode5_command)
{
    return std::vector<float>(g_m5_cmd.begin(), g_m5_cmd.end());
}
// 클립 미래 미리보기 (학습 `motion` 그룹: preview_on 1 · preview_k1 35 · preview 700). history 1.
REGISTER_OBSERVATION(motion_preview_on)
{
    return std::vector<float>{g_motion_block[0]};
}
REGISTER_OBSERVATION(motion_preview_k1)
{
    return std::vector<float>(g_motion_block.begin() + 1, g_motion_block.begin() + 1 + g1::preview::STEP);
}
REGISTER_OBSERVATION(motion_preview)
{
    return std::vector<float>(g_motion_block.begin() + 1 + g1::preview::STEP, g_motion_block.end());
}

// yaw-local base velocity command [vx, vy, wz]. The controller (g_loco, updated once per
// step) splines this across mode switches and zeroes it where !base_vel_live. Raw joystick
// target is computed by g_joystick_base_vel() and fed to g_loco.update() in policy_thread.
REGISTER_OBSERVATION(base_vel_command)
{
    return std::vector<float>(g_loco.base_vel.begin(), g_loco.base_vel.end());
}

// Reference foot height [z_L, z_R]. MUST be the LAST policy obs term (after base_vel + mask),
// matching the trained obs order.
//
// 🔴 원천은 «모드마다 다르다» — mjlab_g1_motion g1_mimic_env.calc_ref_foot_height 그대로:
//     if loco_controller and use_foot_gen:  return loco_controller.foot_z   # 생성기
//     else:                                 return mc.body_pos_w[:, feet, 2]  # 레퍼런스 발 world-z
//   그 분기를 가르는 것은 학습 cfg 에 FOOT_GEN 이 있느냐다.
//     mode1 (stage4_mode1_env_cfg FOOT_GEN foot_source="lut") -> 생성기
//     mode2 (stage4_mode2_env_cfg FOOT_GEN foot_source="lut") -> 생성기
//     mode3 (stage4_mode3_env_cfg 에 FOOT_GEN «없음»)         -> 레퍼런스 발 world-z
//   그 «학습 사실» 이 표의 foot_z 칸이다 — 이 함수는 번호가 아니라 그 칸을 본다.
//   클립 재생 모드(mode4)는 mode3 과 같은 head 를 쓰므로 같은 Ref 다.
//
// 종전 배포는 모든 모드에서 생성기를 냈다. Ref 인 모드는 base_vel 이 0 으로 마스킹돼 eff=0 →
// 서있기 게이트 → 항상 {stance_z, stance_z} 상수. 즉 클립/VR 이 발을 들어 올리는 동안에도
// 정책에겐 «두 발 접지» 라고 말하고 있었다 = 학습과 정면으로 다른 obs.
REGISTER_OBSERVATION(ref_foot_height)
{
    // foot_z: none = 학습에서 foot_z_live=False → 발-z 명령 두 칸은 «정확히 0» 이다(생성기 stance 값이 아니다).
    if (g_mode.row().foot_z == mode_table::FootZ::None) return std::vector<float>{0.f, 0.f};
    if (g_footz_src != FootZSrc::Gen && g_mode.row().foot_z == mode_table::FootZ::Ref) {
        auto loader = active_ref_loader();     // 레퍼런스 발 z 가 원천인 모드
        bool have = false;
        std::array<float, 2> z = {0.f, 0.f};
        if (g_ref_is_clip()) {                  // 클립 재생
            if (loader->has_foot_z) { z = loader->foot_z_clip(); have = true; }
        } else if (loader->vr_override && loader->vr_has_foot_z) {   // VR live
            z = loader->vr_foot_z; have = true;
        }
        if (have) {
            if (g_footz_src == FootZSrc::Ramp) {
                // 다리 q_ref 와 «같은 시계»로 stance -> 레퍼런스. alpha=1 이면 no-op.
                const float a = g_loco.switch_alpha;
                const float s0 = g_loco.foot_z[0], s1 = g_loco.foot_z[1];   // 명령을 안 쓰는 모드에선 stance 상수
                z = { s0 + a * (z[0] - s0), s1 + a * (z[1] - s1) };
            }
            return std::vector<float>{ z[0], z[1] };
        }
        // 폴백: VR 미접속 standby, 또는 발-z 를 못 얻은 클립/구버전 publisher.
        // 이때 레퍼런스는 «기본 서있는 자세» 이므로 두 발 접지가 맞고, 그 값이 곧
        // 아래 g_loco.foot_z (명령을 안 쓰는 모드에선 항상 {stance_z, stance_z}) 다.
    }
    return std::vector<float>(g_loco.foot_z.begin(), g_loco.foot_z.end());
}

// pelvis anchor orientation error as Rot6D in robot base frame. mjlab anchor = PELVIS (=root),
// NOT torso (so no waist-joint offset, unlike motion_anchor_ori_b). 상체를 안 추종하는 모드에선 0.
REGISTER_OBSERVATION(masked_root_ori_b)
{
    std::vector<float> out(6, 0.0f);
    if (g_mode.row().track_upper) {
        auto loader = active_ref_loader();
        Eigen::Quaternionf real_quat_w = env->robot->data.root_quat_w;   // pelvis (IMU)
        Eigen::Quaternionf aligned;
        if (g_ref_is_clip()) {                       // 클립 재생: clip pelvis ori (enter-aligned)
            aligned = init_quat * loader->root_quaternion_clip();
        } else if (loader->vr_override) {            // VR live: VR pelvis ori (enter-aligned)
            aligned = init_quat * loader->root_quaternion_vr();
        } else {
            // standby (상체를 추종하는데 VR 이 없을 때): reference = upright at the robot's CURRENT heading.
            // Bypass init_quat (which is pinned to the ENTER heading) so these modes HOLD the current
            // heading instead of snapping back to where Mimic_Masked was entered. Keeps pitch/roll
            // feedback (only yaw is canceled) -> balance preserved, heading free.
            aligned = isaaclab::yawQuaternion(real_quat_w);
        }
        auto rot_ = aligned.conjugate() * real_quat_w;
        // ⚠ auto 금지: toRotationMatrix() 는 Matrix3f 를 값으로 반환 -> .transpose() 는 그 임시를
        //   참조만 하는 표현식이라 auto 로 받으면 임시 소멸 후 해제된 스택을 읽는다(UB).
        //   -O0 에선 우연히 맞고 -O2/-O3 에선 쓰레기. 실기(-O3)에서만 터진 원인.
        const Eigen::Matrix3f rot = rot_.toRotationMatrix().transpose();
        out = { rot(0,0), rot(0,1), rot(1,0), rot(1,1), rot(2,0), rot(2,1) };
    }
    return out;
}

// pelvis anchor POSITION error in robot base frame. x,y kept only in mode3 (mask_lower),
// z kept in modes 2,3 (mask_upper).
// ⚠ SIM2REAL OBSERVABILITY: real G1 LowState has NO global position (only IMU orientation).
// mode1 -> [0,0,0] EXACT (matches training, all generated). mode2 z (height) and mode3 x,y need
// odometry/height estimation not yet wired -> currently 0 (placeholder). See plan Phase 3b.
REGISTER_OBSERVATION(masked_root_pos_b)
{
    return std::vector<float>(3, 0.0f);
}

}
}


namespace {
// 후행 '/' 를 뗀다. std::filesystem 은 "a/b/" 의 parent_path() 를 "a/b" 로, filename() 을
// «빈 문자열» 로 준다 — config.yaml 의 policy_dir 이 '/' 로 끝나기 때문에 이 함정에 세 번
// 빠졌다(슬롯이 자기 안에 중첩 · slot_name 이 빈 문자열 · 슬롯 목록이 빈 목록).
inline std::filesystem::path trim_sep(std::filesystem::path p)
{
    std::string s = p.string();
    while (!s.empty() && (s.back() == '/' || s.back() == '\\')) s.pop_back();
    return std::filesystem::path(s);
}
// 기동 실패로 즉시 죽는다. std::exit 은 정적 소멸자를 돌리는데, 이 시점엔 아직 join 안 된
// 스레드가 있어 «terminate called without an active exception» 으로 abort(core dump)된다.
// 조작자에겐 위의 critical 메시지가 전부이므로, 로그만 비우고 깨끗한 종료코드로 끝낸다.
[[noreturn]] inline void die_startup()
{
    spdlog::default_logger()->flush();
    std::_Exit(1);
}
}  // namespace

State_Mimic::State_Mimic(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    // 🔴 슬롯 전환 — 하루에 여러 정책을 시험할 때 «파일을 안 고치고» 바꾼다.
    //    G1_POLICY_SLOT=<슬롯이름> 이면 그 슬롯을, 없으면 config.yaml 의 policy_dir 을 쓴다.
    //    왜 환경변수인가: ① 로봇에서 코드·설정을 고치지 않는다(한 방향 규칙) ② 재빌드·pull 이
    //    없다 ③ param.h(공용 base, 6대 공유)를 안 건드린다. 같은 파일이 이미 G1_SAFETY_CSV ·
    //    G1_STATE_CSV · G1_FOOTZ_SRC 를 같은 방식으로 쓴다.
    //    🔴 «Mimic_Masked 에만» 적용한다. State_Mimic 은 Mimic_Dance1_subject2 로도 생성되는데
    //    (policy_dir = config/policy/mimic/dance1_subject2/), 거기까지 가로채면 슬롯 이름을
    //    엉뚱한 부모(mimic/dance1_subject2/)에 붙여 «없는 경로» 를 만든다. 실제로 그랬다 —
    //    테스트가 잡았다. 슬롯 시험의 대상은 multihead 정책 하나뿐이다.
    std::string slot_name;
    std::string policy_cfg = cfg["policy_dir"].as<std::string>();
    if (const char* env_slot = std::getenv("G1_POLICY_SLOT")) {
        if (*env_slot && state_string != "Mimic_Masked") {
            spdlog::info("[slot] G1_POLICY_SLOT 은 Mimic_Masked 에만 적용된다 — '{}' 는 config.yaml 대로",
                         state_string);
        } else if (*env_slot) {
            slot_name = env_slot;
            // 슬롯 «이름» 만 주면 되도록 부모(mimic_masked/)를 이어 붙인다.
            // 🔴 config 값이 '/' 로 끝난다 — 그대로 parent_path() 하면 «자기 자신» 이 나와
            //    슬롯이 자기 안에 중첩된다(.../v4_.../v3_...). 후행 구분자를 먼저 떼야 한다.
            //    테스트가 잡았다.
            std::filesystem::path base = trim_sep(policy_cfg).parent_path();
            policy_cfg = (base / slot_name).string() + "/";
            spdlog::warn("[slot] G1_POLICY_SLOT='{}' -> {} (config.yaml 대신 이것을 쓴다)",
                         slot_name, policy_cfg);
        }
    }
    // 🔴 존재 확인은 parser_policy_dir «앞» 에서 한다 — 그 함수는 exported/ 가 없으면
    //    디렉터리를 순회하는데, 경로 자체가 없으면 filesystem_error 를 던져 abort 된다
    //    (오타 하나에 core dump = 「내가 뭘 잘못 쳤나」를 알 수 없다). 테스트가 잡았다.
    {
        std::filesystem::path probe = policy_cfg;
        if (probe.is_relative()) probe = param::proj_dir / probe;
        if (!std::filesystem::exists(probe)) {
            spdlog::critical("[slot] 그런 슬롯이 없다: {}", probe.string());
            spdlog::critical("       G1_POLICY_SLOT='{}' 오타이거나, 아직 push/pull 이 안 된 슬롯이다.",
                             slot_name.empty() ? std::string("(미지정)") : slot_name);
            spdlog::critical("       있는 슬롯 목록:");
            std::error_code ec;
            std::filesystem::path parent = trim_sep(probe).parent_path();
            for (const auto& e : std::filesystem::directory_iterator(parent, ec))
                if (e.is_directory() && std::filesystem::exists(e.path() / "exported"))
                    spdlog::critical("         - {}", e.path().filename().string());
            die_startup();
        }
    }
    auto policy_dir = param::parser_policy_dir(policy_cfg);
    // 같은 이유로 filename() 도 빈 문자열이 된다 -> 후행 구분자를 뗀 뒤 이름을 뽑는다.
    if (slot_name.empty()) slot_name = trim_sep(policy_dir).filename().string();

    // 🔴 파생을 쓴다 — env->step() 이 obs 직전에 robot->update() 를 «한 번 더» 부르므로
    //    루프에서 후처리하면 덮인다. update() 안에서 보정해야 어느 경로로든 따라간다.
    auto articulation = std::make_shared<g1::G1Articulation<LowState_t::SharedPtr>>(FSMState::lowstate);

    std::filesystem::path motion_file = cfg["motion_file"].as<std::string>();
    if(!motion_file.is_absolute()) {
        motion_file = param::proj_dir / motion_file;
    }

    // Motion
    motion_ = std::make_shared<MotionLoader_>(motion_file.string());
    spdlog::info("Loaded motion file '{}' with duration {:.2f}s", motion_file.stem().string(), motion_->duration);
    motion = motion_;
    // 선택 클립 파일 경로 — 아래 미리보기 기동 검사가 «어느 파일» 인지 말하려고 담아 둔다.
    std::string light_path, demo6_path;

    // 선택 클립: 서기 + 상체 test (light). 없으면 그 칸이 안 생긴다 — 클립 칸은 재생 모드에서
    // 키 '[' / ']' 로 고른다(0 = 주 클립).
    if (cfg["motion_file_light"]) {
        std::filesystem::path light_file = cfg["motion_file_light"].as<std::string>();
        if (!light_file.is_absolute()) light_file = param::proj_dir / light_file;
        motion_light_ = std::make_shared<MotionLoader_>(light_file.string());
        motion_light = motion_light_;
        light_path = light_file.string();
        spdlog::info("Loaded light-demo clip '{}' with duration {:.2f}s",
                     light_file.stem().string(), motion_light_->duration);
    }
    // 선택 클립: 추가 데모(demo6). 이 칸만 «들어올 때마다 처음부터» 다시 튼다.
    // ⚠ 이 클립은 «배포 중인 정책이 학습한 것»이어야 한다 — 슬롯의 ONNX_META manifest 에
    //   없는 클립을 재생하면 OOD 라 실기에서 낙상이다.
    if (cfg["motion_file_demo6"]) {
        std::filesystem::path demo6_file = cfg["motion_file_demo6"].as<std::string>();
        if (!demo6_file.is_absolute()) demo6_file = param::proj_dir / demo6_file;
        motion_demo6_ = std::make_shared<MotionLoader_>(demo6_file.string());
        motion_demo6 = motion_demo6_;
        demo6_path = demo6_file.string();
        spdlog::info("Loaded demo6 clip '{}' with duration {:.2f}s",
                     demo6_file.stem().string(), motion_demo6_->duration);
    }
    if(cfg["time_start"]) {
        float time_start = cfg["time_start"].as<float>();
        time_range_[0] = std::clamp(time_start, 0.0f, motion_->duration);
    } else {
        time_range_[0] = 0.0f;
    }
    if(cfg["time_end"]) {
        float time_end = cfg["time_end"].as<float>();
        time_range_[1] = std::clamp(time_end, 0.0f, motion_->duration);
    } else {
        time_range_[1] = motion_->duration;
    }
    std::string end_state = "Velocity";
    if (cfg["end_state"]) {
        end_state = cfg["end_state"].as<std::string>();
    }
    // mode-switch crossfade window (ticks @50Hz). Tune on hardware without recompiling: edit
    // config.yaml + restart g1_ctrl. Default 50 = 1.0s (0=off/instant, 25=0.5s).
    if (cfg["switch_blend_steps"]) {
        g_loco.switch_blend_steps = cfg["switch_blend_steps"].as<int>();
        spdlog::info("mode-switch crossfade window = {} ticks (~{:.2f}s @50Hz)",
                     g_loco.switch_blend_steps, g_loco.switch_blend_steps / 50.0f);
    }

    auto dcfg = YAML::LoadFile(policy_dir / "params" / "deploy.yaml");
    // 🔴 config 계약: 이 슬롯이 요구하는 C++ 기능을 이 바이너리가 아는가.
    //    yaml 파서는 «모르는 키를 그냥 지나친다» — 그래서 옛 바이너리로 새 슬롯을 돌리면
    //    에러 없이 옛 거동이 나온다(2026-08-26 gait table/cadence/turn_asym 이 그럴 뻔했다).
    //    obs 계약과 같은 자리에서 같은 방식으로 죽인다. requires: 가 없는 옛 슬롯은 통과.
    if (dcfg["requires"] && dcfg["requires"].IsSequence()) {
        std::vector<std::string> req;
        for (const auto& n : dcfg["requires"]) req.push_back(n.as<std::string>());
        const auto miss = g1_features::missing(req);
        if (!miss.empty()) {
            spdlog::critical("[deploy 계약] {}", g1_features::explain(miss, slot_name));
            die_startup();
        }
        std::string joined;
        for (const auto& f : req) joined += (joined.empty() ? "" : " ") + f;
        spdlog::info("[deploy 계약] 슬롯 '{}' 요구 기능 {}개 전부 지원: {}",
                     slot_name, req.size(), joined);
    } else {
        spdlog::info("[deploy 계약] 슬롯 '{}' 은 requires: 를 선언하지 않았다 (옛 슬롯 — 검사 생략)",
                     slot_name);
    }
    // 이 슬롯(ONNX)이 아는 모드. 없으면 계약 v1 = {1,2,3,4}.
    // 🔴 여기서는 «읽어 담기만» 한다 — 거는 것은 enter() 다. 이 클래스는 FSM 상태마다 하나씩
    //    만들어지므로(Mimic_Dance1_subject2 / Mimic_Masked) 생성자에서 전역에 걸면 나중에
    //    만들어진 쪽이 남의 집합을 조용히 덮어쓴다(g_build_clips 와 같은 함정).
    // 🔴 검증하고 죽인다 — requires: 와 같은 자리, 같은 방식(critical + die_startup). 이유 셋:
    //    ① n.as<int>() 는 오타(`modes: [1, 2, tree]`)에 YAML::TypedBadConversion 을 던지는데 이
    //       생성자는 CtrlFSM(YAML) -> main() 까지 catch 가 하나도 없다 = std::terminate/abort.
    //       조작자에겐 core dump 뿐이고 «어느 슬롯의 무슨 값이 틀렸나» 가 안 남는다.
    //    ② 폴백 모드가 빠진 목록(또는 `modes: []`)은 qd 안전 래치를 «영원히» 못 풀게 만든다 —
    //       force() 는 supports() 를 안 보지만 request() 는 보므로, 조작자가 키를 눌러도
    //       requested() 가 폴백으로 안 바뀐다 (rules/ADDING_A_MODE.md 함정 (a)).
    //    ③ 표에 없는 번호는 row() 가 조용히 첫 행으로 바꿔 «아는 모드» 인 척한다.
    if (dcfg["modes"]) {
        if (!dcfg["modes"].IsSequence()) {
            spdlog::critical("[mode] 슬롯 '{}' 의 deploy.yaml 에 modes: 가 목록이 아니다 — `modes: [1, 2, 3, 4]` 형식이어야 한다",
                             slot_name);
            die_startup();
        }
        std::vector<int> ms;
        for (const auto& n : dcfg["modes"]) {
            int m = 0;
            try {
                m = n.as<int>();
            } catch (const std::exception& e) {
                spdlog::critical("[mode] 슬롯 '{}' 의 modes: 에 정수가 아닌 값이 있다: '{}' ({})",
                                 slot_name, n.Scalar(), e.what());
                die_startup();
            }
            if (!mode_table::valid(m)) {
                spdlog::critical("[mode] 슬롯 '{}' 의 modes: 에 표에 없는 모드 {} 가 있다 (표는 1~{}, config/modes.yaml)",
                                 slot_name, m, mode_table::N_MODES);
                die_startup();
            }
            ms.push_back(m);
        }
        if (std::find(ms.begin(), ms.end(), G1_FALLBACK_MODE) == ms.end()) {
            spdlog::critical("[mode] 슬롯 '{}' 의 modes: 에 폴백 모드 {} 가 없다 — qd 안전 래치를 조작자가 풀 수 없다 "
                             "(rules/ADDING_A_MODE.md 함정 (a))", slot_name, G1_FALLBACK_MODE);
            die_startup();
        }
        slot_modes_ = ms;
    }
    {
        std::string s; for (int m : slot_modes_) s += std::to_string(m) + " ";
        spdlog::info("[mode] {} (슬롯 '{}') 이 아는 모드: {}  — 거는 것은 enter()",
                     state_string, slot_name, s);
    }
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        dcfg,
        articulation
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");
    // 🔴 obs 계약을 «모터가 돌기 전에» 대조한다. deploy.yaml 의 항 합이 ONNX 입력과 다르면
    //    act() 가 obs 버퍼를 ONNX 개수만큼 읽어 범위 밖을 건드리거나(작을 때) 조용히
    //    자른다(클 때) — 둘 다 에러 없이 「정책이 이상하다」로만 보인다. 여기서 죽인다.
    //    (lowcmd 인터록과 같은 방식으로 읽히는 메시지 + exit(1). 우회 플래그는 두지 않는다.)
    try {
        // deploy.yaml 이 선언한 항 목록을 그대로 넘긴다 (선언 순서 = obs 배치 순서).
        std::vector<isaaclab::ObsTermSpec> obs_terms;
        for (const auto& t : env->observation_manager->group_terms("obs"))
            obs_terms.push_back({t.name, t.train_name, t.dim(), t.history_length});
        // 🔴 «선언» 을 대조한다 — compute() 가 아니다. 여기는 State::enter() 이전이라
        //    motion 이 아직 안 붙었고, masked_joint_command 같은 항은 빈 벡터를 낸다.
        //    compute() 로 재면 1640 대신 1060 이 나와 멀쩡한 정책이 기동 거부된다
        //    (2026-08-26 sim2sim 에서 실제로 발생). 폭은 deploy.yaml 이 이미 선언하고
        //    있고 그것이 ONNX 와 맞는지가 이 검사의 질문이다. 실행 중 실제 크기는
        //    OrtRunner::act() 의 크기 가드가 매 스텝 따로 지킨다.
        std::unordered_map<std::string, std::vector<float>> declared;
        for (const auto& kv : env->observation_manager->declared_sizes())
            declared[kv.first] = std::vector<float>(kv.second);
        env->alg->verify_inputs(declared, obs_terms);
    } catch (const std::exception& e) {
        spdlog::critical("[obs contract] {}", e.what());
        spdlog::critical("  정책: {}", (policy_dir / "exported" / "policy.onnx").string());
        spdlog::critical("  계약: {}", (policy_dir / "params" / "deploy.yaml").string());
        die_startup();
    }
    // 이 슬롯이 계약 v2 미리보기 항을 선언했나. 담기만 하고 거는 것은 enter() (slot_modes_ 와 같은 이유).
    for (const auto& t : env->observation_manager->group_terms("obs"))
        if (t.name == "motion_preview_on" || t.name == "motion_preview_k1" || t.name == "motion_preview")
            obs_has_preview_ = true;
    spdlog::info("[obs contract] {} (슬롯 '{}') 클립 미리보기 항: {}", state_string, slot_name,
                 obs_has_preview_ ? "있음 (계약 v2 — 미리보기 모드의 클립은 골반 선·각속도가 필요)"
                                  : "없음 (미리보기를 계산하지 않는다)");
    // 🔴 미리보기 항을 선언한 슬롯이면 이 인스턴스의 모든 클립이 골반 선·각속도를 가져야 한다. 없으면
    //    mode4 미리보기가 0(=«해당 없음»)으로 나가 학습과 다른 입력이 된다. 여기서 죽인다 — 생성자는
    //    모터가 PD 에 물리기 «전» 이다. enter() 는 이미 gain 을 건 뒤라 거기서 _Exit 하면 lowcmd 가 끊긴다.
    //    (enter() 가 짓는 클립 칸 g_clips 는 이 세 로더다 — 다른 인스턴스는 선택 클립을 안 싣는다.)
    if (obs_has_preview_) {
        auto need_preview = [&](const std::shared_ptr<MotionLoader_>& l, const char* name, const std::string& path) {
            if (!l || l->has_preview) return;
            spdlog::critical("[obs contract] 슬롯 '{}' 은 클립 미리보기 항을 선언했는데 클립 «{}» ({}) 에 "
                             "body_lin_vel_w·body_ang_vel_w 가 없다(또는 모양이 다르다) — 기동을 거부한다",
                             slot_name, name, path);
            die_startup();
        };
        need_preview(motion_, "primary", motion_file.string());
        need_preview(motion_light_, "light", light_path);
        need_preview(motion_demo6_, "demo6", demo6_path);
    }
    load_safety_cfg(dcfg["safety"]);   // fail-safe: 없거나/이상하면 전부 비활성 (아래 정의)
    load_gait_cfg(dcfg["gait"]);       // fail-safe: 없으면 종전 quintic 기본값 유지 (아래 정의)
    g_load_footz_src();                // 🔬 진단 A/B (env G1_FOOTZ_SRC): ref | gen | ramp

    const auto & joy = FSMState::lowstate->joystick;
    // end_state가 자기자신이면(=masked/teleop) 클립 끝이 무의미 → 타임아웃 체크 미등록 = 무한 실행.
    // (Dance1 등 데모 clip은 end_state 기본값 "Velocity" != 자기이름 → 등록되어 1회 재생 후 복귀 유지.)
    if (end_state != state_string) {
        this->registered_checks.emplace_back(
            std::make_pair(
                [&]()->bool{ return (env->episode_length * env->step_dt) > time_range_[1]; }, // time out
                FSMStringMap.right.at(end_state)
            )
        );
    }
    this->registered_checks.emplace_back(
        std::make_pair(
            [this]()->bool{ // bad orientation
                // 판정은 bad_orientation() 단일 출처. tilt 는 로그·최댓값 기록용으로만 다시 구한다.
                const auto & g = env->robot->data.projected_gravity_b;
                const float tilt_deg = std::acos(std::clamp(-g[2], -1.0f, 1.0f)) * 57.29578f;
                if (tilt_deg > mon_tilt_max_deg_) mon_tilt_max_deg_ = tilt_deg;
                // GroundCapable 모드는 «명령=직립 ∧ 최근 1 s 에 섰음» 일 때만 판정한다(SafetyPolicy.h) —
                // 관문은 정책 스레드가 매 틱 계산해 orient_gate_ 에 둔다. UprightOnly 는 종전 그대로(항상 판정).
                // 한계는 SafetyPolicy.h 의 ORIENT_TRIP_RAD(1.0 rad) — «섰음» 기억이 같은 한계를 쓴다.
                const bool trip = isaaclab::mdp::bad_orientation(env.get(), g1::safety::ORIENT_TRIP_RAD) && orient_gate_.load();
                if (trip && !mon_exit_reason_) {
                    mon_exit_reason_ = "bad_orientation";
                    spdlog::warn("[safety] bad_orientation TRIPPED  tilt={:.1f}deg (limit 57.3) -> Passive", tilt_deg);
                    safety_log_.event("bad_orientation", -1, "", tilt_deg, 57.3f, "-> Passive");
                }
                return trip;
            },
            FSMStringMap.right.at("Passive")
        )
    );
    this->registered_checks.emplace_back(
        std::make_pair(
            [this]()->bool{ // qd crit -> Passive
                const bool trip = js_enable_qd_guard_ && js_qd_crit_latched_.load();
                if (trip && !mon_exit_reason_) mon_exit_reason_ = "qd_crit";
                return trip;
            },
            FSMStringMap.right.at("Passive")
        )
    );
    this->registered_checks.emplace_back(
        std::make_pair(
            [this]()->bool{ // 바닥 자세 qd_warn -> Passive
                const bool trip = js_lowpose_passive_.load();
                if (trip && !mon_exit_reason_) mon_exit_reason_ = "qd_warn_lowpose";   // 체류 요약이 «operator» 로 적지 않게
                return trip;
            },
            FSMStringMap.right.at("Passive")
        )
    );
}

// deploy 관절 순서 29 (legL/legR/waist/armL/armR) — 로그를 사람이 읽게 하기 위한 이름표.
static const char* const G1_JOINT_NAME[29] = {
    "L_hip_pitch","L_hip_roll","L_hip_yaw","L_knee","L_ank_pitch","L_ank_roll",
    "R_hip_pitch","R_hip_roll","R_hip_yaw","R_knee","R_ank_pitch","R_ank_roll",
    "waist_yaw","waist_roll","waist_pitch",
    "L_sho_pitch","L_sho_roll","L_sho_yaw","L_elbow","L_wri_roll","L_wri_pitch","L_wri_yaw",
    "R_sho_pitch","R_sho_roll","R_sho_yaw","R_elbow","R_wri_roll","R_wri_pitch","R_wri_yaw"};
static inline const char* jname(int i) { return (i >= 0 && i < 29) ? G1_JOINT_NAME[i] : "?"; }

// 안전층이 값을 깎았는지 전/후 비교로 집계. NaN 이면 fabs(NaN)>mx 가 false 라 조용히 건너뛴다.
static inline void mon_track(const float* pre, const float* post, int n,
                             uint32_t& ticks, float& max_d, int& max_j) {
    float mx = 0.0f; int mj = -1;
    for (int i = 0; i < n; ++i) { float d = std::fabs(post[i] - pre[i]); if (d > mx) { mx = d; mj = i; } }
    if (mj >= 0) { ++ticks; if (mx > max_d) { max_d = mx; max_j = mj; } }
}

void State_Mimic::mon_reset()
{
    mon_clamp_ticks_ = mon_rate_ticks_ = 0;
    mon_nan_hold_ticks_ = 0;
    mon_clamp_max_ = mon_rate_max_ = mon_qd_max_ = mon_tilt_max_deg_ = 0.0f;
    mon_clamp_joint_ = mon_rate_joint_ = mon_qd_joint_ = -1;
    mon_exit_reason_ = nullptr;
}

// 체류 요약 — 실기 발산은 0.2s 라 눈으로 못 본다. 나간 뒤 이 한 덩어리가 유일한 증거다.
void State_Mimic::mon_log_summary()
{
    const float dwell = env->episode_length * env->step_dt;
    spdlog::info("[Mimic_Masked] 체류 {:.1f}s, 최종 mode{}, 종료사유 {}",
                 dwell, g_mode.mode(), mon_exit_reason_ ? mon_exit_reason_ : "operator");
    if (mon_clamp_joint_ >= 0)
        spdlog::info("  pos_clamp  : {} tick (최대 {:.4f} rad 삭감 @ {} {})",
                     mon_clamp_ticks_, mon_clamp_max_, mon_clamp_joint_, jname(mon_clamp_joint_));
    else
        spdlog::info("  pos_clamp  : {} tick ({})", mon_clamp_ticks_, js_enable_pos_clamp_ ? "무개입" : "OFF");
    if (mon_rate_joint_ >= 0)
        spdlog::info("  rate_limit : {} tick (최대 {:.4f} rad 삭감 @ {} {})",
                     mon_rate_ticks_, mon_rate_max_, mon_rate_joint_, jname(mon_rate_joint_));
    else
        spdlog::info("  rate_limit : {} tick ({})", mon_rate_ticks_, js_enable_rate_limit_ ? "무개입" : "OFF");
    spdlog::info("  nan_hold   : {} tick (출력 NaN/Inf 를 직전값으로 홀드, 항상 ON)", mon_nan_hold_ticks_);
    spdlog::info("  |qd| 최대  : {:.2f} rad/s @ {} {}   (warn {:.1f} / crit {:.1f}, guard {})",
                 mon_qd_max_, mon_qd_joint_, jname(mon_qd_joint_),
                 js_qd_warn_, js_qd_crit_, js_enable_qd_guard_ ? "ON" : "OFF");
    spdlog::info("  기울기 최대: {:.1f} deg  (limit 57.3)", mon_tilt_max_deg_);
}

// deploy.yaml의 safety 블록 로드 (fail-safe): 블록이 없거나, pos_min/pos_max 길이가 29가 아니거나,
// 파싱 중 예외가 나면 무조건 clamp 비활성. 절대 예외를 밖으로 던지지 않고, 절대 0으로 clamp하지 않는다.
void State_Mimic::load_safety_cfg(const YAML::Node& s)
{
    js_enable_pos_clamp_ = false;
    if (!s || !s.IsMap()) { spdlog::warn("[safety] no safety block -> disabled"); return; }
    try {
        auto lo = s["pos_min"].as<std::vector<float>>();
        auto hi = s["pos_max"].as<std::vector<float>>();
        if (lo.size()!=29 || hi.size()!=29) { spdlog::warn("[safety] pos_min/max len!=29 -> clamp disabled"); return; }
        for (int i=0;i<29;++i){ js_pos_lo_[i]=lo[i]; js_pos_hi_[i]=hi[i]; }
        // std::clamp(v,lo,hi)는 lo>hi면 UB -> per-joint 순서 검증 없이는 무장 금지 (config 오타 방어).
        for (int i=0;i<29;++i) {
            if (js_pos_lo_[i] >= js_pos_hi_[i]) {
                spdlog::warn("[safety] pos_min[{}]>=pos_max[{}] -> clamp disabled", i, i);
                js_enable_pos_clamp_ = false;
                return;
            }
        }
        js_enable_pos_clamp_ = s["enable_pos_clamp"] && s["enable_pos_clamp"].as<bool>();
        spdlog::info("[safety] pos_clamp {} (mechanical limits loaded)", js_enable_pos_clamp_?"ON":"OFF");
    } catch (const std::exception& e) {
        js_enable_pos_clamp_ = false;
        spdlog::warn("[safety] parse error -> clamp disabled: {}", e.what());
    }

    // L2: 속도 rate-limit용 vel_max 파싱 (fail-safe). 없거나/0이하/파싱 오류 -> 비활성.
    js_enable_rate_limit_ = false;
    // 스칼라(전 관절 공통) 또는 29개 리스트(관절별). 그 외 길이는 파싱 오류 = 비활성.
    //   2026-09-09: 관절별 = 하드웨어 최고속도의 90 %. 스칼라 15 는 hip pitch(hw 32)를 무릎(hw 20)
    //   값에 묶어 러닝 명령을 11~25 % 잘랐다 (260907 판독). 관절별 리스트는 «safety_per_joint» 기능이
    //   있는 바이너리만 읽는다 — 옛 바이너리는 리스트를 float 로 못 읽고 층을 «조용히 끄므로» requires 로 거부.
    auto read_per_joint = [&](const char* key, std::array<float,29>& out) -> bool {
        if (!s[key]) return false;
        std::vector<float> v;
        if (s[key].IsSequence()) for (const auto& e : s[key]) v.push_back(e.as<float>());
        else v.push_back(s[key].as<float>());
        return js_expand_per_joint(v.data(), (int)v.size(), out.data(), 29);
    };
    try {
        const bool have = read_per_joint("vel_max", js_vel_max_v_);
        float vmin = 0.f, vmaxv = 0.f;
        if (have) { vmin = *std::min_element(js_vel_max_v_.begin(), js_vel_max_v_.end());
                    vmaxv = *std::max_element(js_vel_max_v_.begin(), js_vel_max_v_.end()); }
        // run()은 CtrlFSM에서 1kHz로 호출된다(정책 step_dt=0.02는 policy_thread 전용). js_rate_limit이
        // 매 FSM tick 적용되므로 dt = 1kHz tick = 0.001. (max_step=vel_max*0.001 -> effective cap = vel_max rad/s;
        // 20ms 정책창당 최대 20*vel_max*0.001 = vel_max*0.02 이동.)
        const float dt = 0.001f;   // CtrlFSM run() tick (1kHz), NOT the 50Hz policy step_dt
        if (have && vmin > 0.0f) {
            for (int i=0;i<29;++i) js_max_step_[i] = js_vel_max_v_[i] * dt;
            js_enable_rate_limit_ = s["enable_rate_limit"] && s["enable_rate_limit"].as<bool>();
        }
        if (vmin == vmaxv)
            spdlog::info("[safety] rate_limit {} (vel_max={} rad/s)", js_enable_rate_limit_?"ON":"OFF", vmin);
        else
            spdlog::info("[safety] rate_limit {} (vel_max={}~{} rad/s 관절별: hip_pitch {} knee {} ankle_pitch {})",
                         js_enable_rate_limit_?"ON":"OFF", vmin, vmaxv, js_vel_max_v_[0], js_vel_max_v_[3], js_vel_max_v_[4]);
    } catch (const std::exception& e) {
        js_enable_rate_limit_ = false;
        spdlog::warn("[safety] rate parse err: {}", e.what());
    }

    // L3: 측정 qd 폭주 가드 파싱 (fail-safe). warn>0 이고 crit>warn 이어야 무장, 그 외엔 비활성.
    js_enable_qd_guard_ = false;
    try {
        const bool hw = read_per_joint("qd_warn", js_qd_warn_v_);
        const bool hc = read_per_joint("qd_crit", js_qd_crit_v_);
        js_over_ticks_ = s["over_ticks"] ? s["over_ticks"].as<int>() : 5;
        if (js_over_ticks_ < 1) js_over_ticks_ = 1;
        // 무장 조건: 모든 관절에서 warn>0 이고 crit>warn. 하나라도 어긋나면 비활성(fail-safe).
        bool ok = hw && hc;
        for (int i=0; ok && i<29; ++i) ok = (js_qd_warn_v_[i] > 0.f) && (js_qd_crit_v_[i] > js_qd_warn_v_[i]);
        js_qd_warn_ = ok ? *std::min_element(js_qd_warn_v_.begin(), js_qd_warn_v_.end()) : 0.f;
        js_qd_crit_ = ok ? *std::min_element(js_qd_crit_v_.begin(), js_qd_crit_v_.end()) : 0.f;
        if (ok) js_enable_qd_guard_ = s["enable_qd_guard"] && s["enable_qd_guard"].as<bool>();
        const float cmax = ok ? *std::max_element(js_qd_crit_v_.begin(), js_qd_crit_v_.end()) : 0.f;
        if (js_qd_crit_ == cmax)
            spdlog::info("[safety] qd_guard {} (warn={} crit={} rad/s, over_ticks={})",
                         js_enable_qd_guard_?"ON":"OFF", js_qd_warn_, js_qd_crit_, js_over_ticks_);
        else
            spdlog::info("[safety] qd_guard {} (warn={} crit={}~{} rad/s 관절별: hip_pitch {} knee {} ankle_pitch {}, over_ticks={})",
                         js_enable_qd_guard_?"ON":"OFF", js_qd_warn_, js_qd_crit_, cmax,
                         js_qd_crit_v_[0], js_qd_crit_v_[3], js_qd_crit_v_[4], js_over_ticks_);
        // 사다리 검사(관절별): warn < vel_max < crit. 어긋나도 끄진 않지만 기동 로그에 남긴다.
        if (ok && js_enable_rate_limit_)
            for (int i=0;i<29;++i)
                if (!(js_qd_warn_v_[i] < js_vel_max_v_[i] && js_vel_max_v_[i] < js_qd_crit_v_[i]))
                    spdlog::warn("[safety] 사다리 어긋남 @ {} {}: warn {} < vel_max {} < crit {} 이 아니다",
                                 i, jname(i), js_qd_warn_v_[i], js_vel_max_v_[i], js_qd_crit_v_[i]);
    } catch (const std::exception& e) {
        js_enable_qd_guard_ = false;
        spdlog::warn("[safety] qd parse err: {}", e.what());
    }
}

// deploy.yaml의 gait 블록 로드 (fail-safe): 없거나 파싱이 실패하면 손대지 않는다 →
// MaskedLocoController 의 기본값(전 모드 quintic / height_scale 1.0 / deadzone 없음)이 남아
// 종전 거동이 그대로 유지된다.
//
// ⚠ 왜 모드별인가: 배포 ONNX 의 head 는 각각 다른 시점에 학습돼 foot_z 생성 조건이 다르다
//    (ONNX_META.json 의 mode{N}_ckpt ↔ 그 시점 stage4_mode{N}_env_cfg.py 의 FOOT_GEN).
//    하나로 통일하면 반드시 어느 head 하나가 학습과 다른 발-z 명령을 받는다. 이 값은 weights
//    에 안 남는 런타임 스칼라라 config 로 받는 수밖에 없고, 정책과 같이 다니도록 policy 쪽
//    deploy.yaml 에 둔다(config.yaml 아님).
// ⚠ g_loco 는 파일 전역이고 State_Mimic 은 두 번 만들어진다(mimic=dance / mimic_masked).
//    그래서 «블록이 없으면 손대지 않는다» 가 중요하다 — gait 블록이 없는 dance 쪽 생성자가
//    나중에 돌아도 mimic_masked 가 실은 값을 지우지 않는다. 두 정책이 서로 다른 gait 블록을
//    갖게 되면 그때는 생성 순서가 승자를 정하므로, 그 시점에 모드표를 인스턴스로 옮길 것.
void State_Mimic::load_gait_cfg(const YAML::Node& g)
{
    if (!g || !g.IsMap()) { spdlog::warn("[gait] no gait block -> 전 모드 quintic 기본값 유지"); return; }
    try {
        if (g["walk_max"]) g_loco.walk_max = g["walk_max"].as<float>();
        if (g["run_min"])  g_loco.run_min  = g["run_min"].as<float>();
        // 정착 파라미터 — 파이썬도 이 둘은 스칼라, settle_steps 만 모드별.
        if (g["settle_eff"])   g_loco.settle_eff   = g["settle_eff"].as<float>();
        if (g["settle_phase"]) g_loco.settle_phase = g["settle_phase"].as<float>();
        for (int m = 1; m <= mode_table::N_MODES; ++m) {
            const YAML::Node n = g[mode_table::row(m).gait_key];
            if (!n || !n.IsMap()) continue;
            auto& mg = g_loco.mode_gait[m];
            const std::string src = n["source"] ? n["source"].as<std::string>() : "quintic";
            mg.lut = (src == "lut");
            // 표 판번호 — 이 head 가 «어느 시점의 구운 표» 로 학습됐나. 미지정 = 1 = 종전 배포 표.
            if (n["table"])     mg.table     = n["table"].as<int>();
            if (n["cadence"])   mg.cadence   = n["cadence"].as<float>();
            if (n["turn_asym"]) mg.turn_asym = n["turn_asym"].as<bool>();
            if (n["settle_steps"]) mg.settle_steps = n["settle_steps"].as<int>();
            if (n["min_swing"])    mg.min_swing    = n["min_swing"].as<float>();
            if (mg.table != 1 && mg.table != 2) {
                spdlog::warn("[gait] mode{}: table={} 은 없는 판번호 -> 1 로 되돌린다", m, mg.table);
                mg.table = 1;
            }
            // LUT 은 표가 stance 높이를 갖고 있다 — 파이썬도 foot_source=="lut" 이면
            // stance_z 를 gait_lut.STANCE_Z 로 덮어쓴다. 같은 순서로 덮고, yaml 이 명시하면 그것이 이긴다.
            // 🔴 «그 모드의 표» 의 stance 다. V1 6.6877 cm ↔ V2 3.5000 cm 로 3.2 cm 차이가 나므로
            //    표를 골라 놓고 stance 를 안 따라가면 발-z obs 가 통째로 어긋난다.
            if (mg.lut) mg.stance_z = MaskedLocoController::table_of(mg).stance_z;
            if (n["stance_z"])       mg.stance_z       = n["stance_z"].as<float>();
            if (n["height_scale"])   mg.height_scale   = n["height_scale"].as<float>();
            if (n["stand_deadzone"]) mg.stand_deadzone = n["stand_deadzone"].as<float>();
            spdlog::info("[gait] mode{}: source={} table=V{} cadence={:.3f} asym={} settle={} "
                         "min_swing={:.3f} height_scale={:.2f} stance_z={:.5f} deadzone={:.2f}",
                         m, mg.lut ? "lut" : "quintic", mg.table, mg.cadence,
                         mg.turn_asym ? "on" : "off", mg.settle_steps, mg.min_swing,
                         mg.height_scale, mg.stance_z, mg.stand_deadzone);
        }
        spdlog::info("[gait] walk_max={:.2f} run_min={:.2f} (lut gait 히스테리시스)",
                     g_loco.walk_max, g_loco.run_min);
    } catch (const std::exception& e) {
        spdlog::warn("[gait] parse error -> 남은 값 그대로 사용: {}", e.what());
    }
}

void State_Mimic::enter()
{
    // set gain
    for (int i = 0; i < env->robot->data.joint_stiffness.size(); i++)
    {
        lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
        lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
        lowcmd->msg_.motor_cmd()[i].dq() = 0;
        lowcmd->msg_.motor_cmd()[i].tau() = 0;
    }

    motion = motion_; // set for specific motion
    // 클립 슬롯은 «여기서» 짓는다 — 생성자가 아니라. State_Mimic 은 두 번 만들어지고(dance /
    // masked) 주 클립(motion)은 enter() 마다 그 인스턴스 것으로 다시 묶이므로, 생성자에서만
    // 지으면 0번 칸이 나중에 만들어진 쪽을 가리킨 채 남는다. 정책 루프가 뜨기 전이다.
    g_build_clips();
    // 같은 이유로 «이 슬롯이 아는 모드» 도 여기서 건다 — 지금 들어가는 정책의 것이어야 한다.
    g_mode.set_supported(slot_modes_);
    // 지난 체류의 모드가 이 슬롯엔 없을 수 있다(예: 계약 v2 에서 mode5 로 나갔다가 v1 슬롯으로 진입).
    // 그대로 두면 ONNX 가 모르는 모드로 첫 틱이 돈다 → 표의 첫 행(안전 폴백)으로 내린다.
    if (!g_mode.supports(g_mode.mode())) {
        spdlog::warn("[mode] 직전 모드 {} 는 이 슬롯이 모른다 -> mode{} 로 내린다",
                     g_mode.mode(), G1_FALLBACK_MODE);
        g_mode.force(G1_FALLBACK_MODE);
    }
    // 미리보기 항 선언도 «지금 들어가는 슬롯» 의 것으로 (생성자가 담아 둔 값). 클립 속도 배열 검사는
    // 생성자가 이미 했다(모터 전 — 여기서 죽이면 PD 가 걸린 채 lowcmd 가 끊긴다).
    g_obs_has_preview = obs_has_preview_;
    // mode5 로 들어오는 또 하나의 길 = FSM 재진입. 모드는 체류를 넘어 남으므로(mode5 에서 p·안전 전이로
    // 나갔다가 다시 m) 정책 루프의 전환 에지가 안 생겨 진입 처리가 안 돈다 → 여기서 같은 규칙을 건다.
    // 안 걸면 지난 체류의 자세(예: 드러누움 hold)가 서 있는 로봇에 그대로 명령된다.
    // 🔴 관측 버퍼도 «여기서» 이 체류의 첫 틱 값으로 맞춘다 — 아래 env->reset()(이 함수 안, 그리고 정책
    //    스레드 시작 때 한 번 더)이 항의 지금 값을 이력 10칸 전부에 복사한다(ObservationTermCfg::reset).
    //    안 맞추면 지난 체류의 마지막 mode5 명령이 첫 0.2 s 이력을 채운다. 사본을 tick 해 진짜 드라이버는
    //    t=0 에 둔다(첫 틱 명령 = 가는 중 · t_goal 0 — 도착 누적 dt < HOLD_S 라 z·g 와 무관).
    if (g_mode.row().mode5_cmd_live) {
        g_m5.press(m5::ENTER_PRESET);
        g_m5_cmd = g1::Mode5Driver(g_m5).tick(g_z_fk, {0.f, 0.f, -1.f});
    } else {
        g_m5.reset();
        g_m5_cmd.fill(0.f);
    }
    std::fill(g_motion_block.begin(), g_motion_block.end(), 0.f);
    // IMU 편향 보정. config.yaml 의 imu_cal 이 원장이고, 환경변수는 «실험용» override 다
    // (sim 에 일부러 편향을 만들어 증상을 재현할 때 쓴다 — G1_IMU_CAL_DEG="4.2,0").
    {
        const YAML::Node ic = param::config["imu_cal"];
        if (ic && ic.IsMap())
            imu_cal_.set(ic["pitch_deg"] ? ic["pitch_deg"].as<float>() : 0.0f,
                         ic["roll_deg"]  ? ic["roll_deg"].as<float>()  : 0.0f);
        // 🔴 sim(lo) 에는 실기 IMU 의 상쇄값을 걸지 않는다 — config.yaml 이 sim·실기 공용이라
        //    여기서 갈라야 한다. 안 가르면 sim 이 «편향 없는 로봇에 틀린 보정» 으로 넘어진다(65°).
        if (imu_cal_.on() && !g1::ImuCal::applies_to_network(g_network_iface)) {
            spdlog::warn("[imu_cal] --network={} (sim) → config 의 보정 {} 을 걸지 않는다. "
                         "sim 에 편향을 만들려면 G1_IMU_CAL_DEG 로.", g_network_iface, imu_cal_.describe());
            imu_cal_.set(0.0f, 0.0f, "lo");
        }
        const bool env_override = imu_cal_.set_from_env();
        spdlog::info("[imu_cal] {}{}", imu_cal_.describe(),
                     env_override ? "  ← 환경변수 override" : "");
        // 실제로 거는 곳은 articulation 이다(update() 마다 적용).
        if (auto* art = dynamic_cast<g1::G1Articulation<LowState_t::SharedPtr>*>(env->robot.get()))
            art->imu_cal = imu_cal_;
        else
            spdlog::error("[imu_cal] 🔴 G1Articulation 이 아니다 — 보정이 걸리지 않는다");
    }
    safety_log_.open_from_env();   // G1_SAFETY_CSV 가 있을 때만 켜진다(기본 꺼짐)
    state_dump_.open_from_env("G1_STATE_CSV", GaitAux::header());   // G1_STATE_CSV 가 있을 때만 (sim2sim ↔ 실기 대조 계측)
    std::remove("/dev/shm/g1_vr_ref");   // clear any stale VR ref so it can't hijack on entry
                                         // (a live bridge re-creates it next frame; g_poll_vr picks up new seq)
    g_gui_rebase = true;                 // GUI 파일은 지우지 않는다(GUI 가 다시 안 쓸 수 있다) — 이 체류의 첫 poll 이
                                         // 기준선만 잡는다(g_poll_gui). 정책 스레드가 뜨기 전이라 경합 없음.
    { // mode2/3 hold this neutral pose (robot default) until VR provides a reference — never the clip
        const auto& dj = env->robot->data.default_joint_pos;
        Eigen::VectorXf dpos((int)dj.size());
        for (int i = 0; i < (int)dj.size(); ++i) dpos[i] = dj[i];
        motion->set_standby(dpos);
    }
    env->reset();
    // L2 rate-limit용 q_prev를 측정 pose로 초기화 (레이어를 나중에 켜도 첫 틱 lurch 방지).
    // env->reset() 직후라 joint_pos는 방금 robot->update()로 갱신된 측정값이다.
    for (int i = 0; i < 29; ++i) js_q_prev_[i] = env->robot->data.joint_pos[i];
    js_q_prev_valid_ = true;
    // L3: qd 가드 래치·카운터 리셋 (재진입 시 깨끗이 시작 — 이전 FSM 체류의 래치가 남지 않도록).
    js_qd_crit_latched_ = false;
    js_qd_warn_latched_ = false;
    js_warn_run_ = js_crit_run_ = 0;
    // 안전 규칙(SafetyPolicy.h) 상태도 체류 단위로 — 정책 스레드가 뜨기 전이라 경합 없음.
    // 기울기 필터는 첫 update() 가 원시값으로 시작하게(지난 체류의 걸러진 값이 남지 않게).
    js_lowpose_passive_.store(false);
    orient_gate_.store(true);
    g_tilt.reset();
    g_recent_high.reset();     // 지난 체류의 «섰음» 이 남지 않게 — 첫 틱부터 다시 센다
    mon_reset();               // 모니터링 카운터도 체류 단위로 리셋 (요약이 이번 체류만 담게)
    // 진입 시 «조작자가 요청한 모드» 를 현재 모드와 일치시켜 시작(불일치 방지). 같은 모드 요청은
    // 이탈 조건을 안 타므로 requested 만 맞춰진다.
    (void)g_mode.request(g_mode.mode());
    // Start policy thread
    policy_thread_running = true;
    policy_thread = std::thread([this]{
        using clock = std::chrono::high_resolution_clock;
        // 정책도 RT 로. 안전(FSM 1kHz)보다는 «아래»다 — 정책이 늦으면 행동이 낡을 뿐이고,
        // 안전이 늦으면 관절 가드가 안 돈다. 실패해도 경고만 남기고 종전대로 돈다.
        rtprio::raise_this_thread("정책 50Hz", rtprio::POLICY_50HZ);
        const std::chrono::duration<double> desiredDuration(env->step_dt);
        const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

        // Initialize timing
        const auto start = clock::now();
        auto sleepTill = start + dt;

        motion->reset(env->robot->data, time_range_[0]);   // 0번 칸 = 주 클립(구간 오프셋을 쓴다)
        for (size_t i = 1; i < g_clips.size(); ++i) {      // 나머지 칸: clip anchor = enter heading
            g_clips[i].loader->reset(env->robot->data, 0.0f);
            g_clips[i].t0 = 0.0f;   // env->reset() 이 episode_length 를 0 으로 되돌리므로 원점도 짝을 맞춘다
        }
        auto ref_yaw = isaaclab::yawQuaternion(motion->root_quaternion()).toRotationMatrix();
        auto robot_yaw = isaaclab::yawQuaternion(robot_quat_w(env.get())).toRotationMatrix();
        init_quat = robot_yaw * ref_yaw.transpose();
        env->reset();

        // ── 속도계 (LoopDiag.h). 로봇 동작은 안 바꾼다 — "지금 몇 ms 쓰나"만 본다.
        // 이 루프는 여태 한 바퀴가 몇 ms 인지 «아무도 안 재고 있었다»: sleep_until 뿐이고
        // sleepTill 은 따라잡기도 안 하므로 한 번 밀리면 누적된다. 새 계산(전환 생성기 등)을
        // 얹기 전에 «남은 예산»을 알아야 한다. 파이썬 브릿지엔 같은 [diag] 가 이미 있다.
        LoopDiag diag(env->step_dt * 1000.0, 1.0);
        static const char* DIAG_SEG[6] = {"poll", "safe", "ctrl", "obs", "ort", "act"};
        auto d_prev = clock::now();
        bool d_first = true;
        char d_buf[768];
        std::FILE* d_csv = nullptr;
        if (const char* path = std::getenv("G1_DIAG_CSV")) {
            d_csv = std::fopen(path, "w");
            if (d_csv) { std::fprintf(d_csv, "%s\n", LoopDiag::csv_header()); std::fflush(d_csv); }
            spdlog::info("[diag] CSV -> {} ({})", path, d_csv ? "열림" : "열기 실패");
        }

        // ── 진단용 부하 주입 (G1_DIAG_LOAD 이 있을 때만). 전환 생성기를 «붙이기 전에»
        // 「별도 스레드로 돌리면 되지 않나」를 실측으로 답하기 위한 것이다. 이 repo 에는
        // 우선순위 설정이 하나도 없으므로, 계산 스레드 하나가 50 Hz 정책과 1 kHz 안전 루프를
        // 얼마나 밀어내는지는 «재야» 안다.
        //   G1_DIAG_LOAD=<policy.onnx 경로>[:<몇 틱마다>]      예) .../policy.onnx:5
        // 진짜 ONNX 를 돌린다 — 캐시 거동까지 같아야 답이 답이 되기 때문. 합성 busy-loop 로는
        // 대역폭·캐시 압력이 재현되지 않는다.
        std::thread load_th;
        std::atomic<bool> load_run{false};
        // 🔴 인터록: 실로봇에서는 «절대» 켜지지 않는다. 환경변수는 스크립트·systemd·셸
        // 히스토리로 새기 쉬운데, 켜지면 관절 안전 루프와 다투는 스레드가 하나 더 생긴다.
        // 그래서 주석이 아니라 «코드»로 막는다 — sim2sim(--network=lo) 이 아니면 거부.
        if (std::getenv("G1_DIAG_LOAD") && g_network_iface != "lo") {
            spdlog::error("[diag:load] 🔴 G1_DIAG_LOAD 가 설정돼 있으나 --network={} 이다. "
                          "진단용 부하는 sim2sim(lo) 에서만 허용된다 — 무시한다.", g_network_iface);
        }
        else if (const char* spec = std::getenv("G1_DIAG_LOAD")) {
            std::string sp(spec);
            int every = 1;
            const auto colon = sp.rfind(':');
            if (colon != std::string::npos && sp.find(".onnx") < colon) {
                every = std::max(1, std::atoi(sp.substr(colon + 1).c_str()));
                sp = sp.substr(0, colon);
            }
            load_run = true;
            spdlog::warn("[diag:load] 🔴 진단용 부하 ON — {} 를 {}틱마다. 실운용에서 켜지 말 것.",
                         sp, every);
            load_th = std::thread([sp, every, &load_run, this]{
              // 예외 격리: OrtRunner 생성자는 경로가 틀리면 throw 한다. 스레드 밖으로 나가면
              // std::terminate -> 제어기 «전체»가 죽는다. 진단 도구가 제어기를 죽이면 안 된다.
              try {
                // 계산은 언제나 안전보다 «뒤». 얼마나 뒤로 미는지는 G1_DIAG_LOAD_SCHED 로 고른다:
                //   nice(기본) = SCHED_OTHER nice +10 · idle = SCHED_IDLE(권한 불필요, 더 강함)
                const char* sched = std::getenv("G1_DIAG_LOAD_SCHED");
                if (sched && std::string(sched) == "idle") rtprio::idle_this_thread("진단 부하");
                else                                       rtprio::lower_this_thread("진단 부하", 10);
                isaaclab::OrtRunner rr(sp);
                auto obs = rr.zero_obs();
                LoopDiag ld(env->step_dt * 1000.0 * every, 1.0);
                auto prev = clock::now();
                bool first = true;
                char buf[768];
                static const char* SEG[6] = {"-","-","-","-","-","-"};
                const auto period = std::chrono::duration_cast<clock::duration>(
                    std::chrono::duration<double>(env->step_dt * every));
                auto next = clock::now() + period;
                while (load_run) {
                    const auto t0 = clock::now();
                    rr.act(obs);
                    const auto t1 = clock::now();
                    auto ms = [](clock::time_point a, clock::time_point b) {
                        return std::chrono::duration<double, std::milli>(b - a).count(); };
                    ld.tick(ms(t0, t1), first ? 0.0 : ms(prev, t0));
                    prev = t0; first = false;
                    if (ld.window_closed()) {
                        ld.format(buf, sizeof buf, SEG);
                        if (char* nl = std::strchr(buf, '\n')) *nl = '\0';
                        spdlog::info("[diag:load] {}", buf + 7);
                        ld.reset();
                    }
                    std::this_thread::sleep_until(next);
                    next += period;
                }
              } catch (const std::exception& e) {
                spdlog::error("[diag:load] 부하 스레드 중단: {}", e.what());
              } catch (...) {
                spdlog::error("[diag:load] 부하 스레드 중단 (알 수 없는 예외)");
              }
            });
        }

        while (policy_thread_running)
        {
            const auto d_t0 = clock::now();
            env->robot->update();   // 보정은 G1Articulation::update() 안에서 이미 걸린다
            // 자세 추정(엔코더 + IMU 만): 이탈 조건·mode5 도착·안전 규칙이 같은 값을 본다.
            {
                const auto& rd = env->robot->data;
                g_z_fk = g1::z_fk(rd.joint_pos.data(), rd.root_quat_w);
                g_tilt.update({rd.projected_gravity_b[0], rd.projected_gravity_b[1], rd.projected_gravity_b[2]});
                // 넘어짐 관문(SafetyPolicy.h): 명령 → «섰음» 기억(원시·걸러진 기울기 둘 다) → 관문. 정책 스레드가
                // 여기서만 쓰고, FSM 스레드의 bad_orientation 람다는 orient_gate_ 원자값만 읽는다.
                // 기억은 직립 모드(UprightOnly)에서도 이어 채운다 — 넘어지는 도중 4·5 로 바꿔도 판정이 걸리게.
                // 관문 식 자체는 commanded_upright 그대로(UprightOnly 는 어차피 항상 적용).
                const mode_table::Safety safety = g_mode.row().safety;
                const bool cmd_upright = g_commanded_upright();
                g_recent_high.update(g_z_fk, g_tilt.raw(), g_tilt.value(),
                                     g1::safety::upright_for_memory(safety, cmd_upright));
                orient_gate_.store(g1::safety::orientation_check_applies(safety, cmd_upright, g_recent_high.value()));
            }
            g_poll_inputs(env.get());   // joystick d-pad + keyboard (모드 키 + WASD/QE vel)
            g_poll_vr();                // VR teleop ref (overrides obs/base_vel/mode if active)
            const auto d_t_poll = clock::now();
            // ── L3: 측정 qd 폭주 감지 (policy_thread 50Hz — g_mode/notify_mode_switch 같은 스레드) ──
            // 모니터링: |qd| 최댓값은 guard on/off 와 무관하게 항상 추적. 50Hz, I/O 없음.
            // qd_now/qd_j 는 아래 warn/crit 로그가 "지금 값"을 찍도록 재사용한다(체류 최댓값 아님).
            const int qd_j = js_qd_argmax(env->robot->data.joint_vel.data(),
                                          (int)env->robot->data.joint_vel.size());
            const float qd_now = (qd_j >= 0) ? std::fabs(env->robot->data.joint_vel[qd_j]) : 0.0f;
            if (qd_j >= 0 && std::isfinite(qd_now) && qd_now > mon_qd_max_) {
                mon_qd_max_ = qd_now; mon_qd_joint_ = qd_j;
            }
            if (js_enable_qd_guard_) {
                const int reqd = g_mode.requested();   // 조작자 실제 요청 모드(가드가 강제한 현재 모드와 분리; 요청이 받아들여질 때만 갱신)
                int sev = js_qd_severity_v(env->robot->data.joint_vel.data(),
                                           std::min((int)env->robot->data.joint_vel.size(), 29),
                                           js_qd_warn_v_.data(), js_qd_crit_v_.data());
                const bool warn_before = js_qd_warn_latched_;
                bool crit_l = js_qd_crit_latched_.load();
                const bool crit_before = crit_l;
                js_qd_step(sev, js_over_ticks_, js_warn_run_, js_crit_run_, js_qd_warn_latched_, crit_l);
                if (crit_l) js_qd_crit_latched_.store(true);
                if (!crit_before && crit_l)     // 래치되는 에지에서 1회만
                    spdlog::error("[safety] qd_crit LATCHED  |qd|={:.2f} rad/s @ {} {} (crit {:.1f} 을 {}틱 연속 초과) -> Passive",
                                  qd_now, qd_j, jname(qd_j), (qd_j>=0&&qd_j<29)?js_qd_crit_v_[qd_j]:js_qd_crit_, js_over_ticks_);
                if (!crit_before && crit_l)
                    safety_log_.event("qd_crit", qd_j, jname(qd_j), qd_now, (qd_j>=0&&qd_j<29)?js_qd_crit_v_[qd_j]:js_qd_crit_, "-> Passive");
                // warn 수동복귀: 조작자가 폴백 모드(X/'1')를 명시하면 해제(qd 아직 높으면 다음 sustained서 재래치).
                if (js_qd_warn_latched_ && reqd == G1_FALLBACK_MODE) js_qd_warn_latched_ = false;
                // g_poll_vr 뒤에 덮어써 폴백 모드 유지(soft).
                if (js_qd_warn_latched_) {
                    // 처분은 «그 순간의 모드» 행으로 정한다(SafetyPolicy.h qd_warn_action): UprightOnly 는 종전
                    // 그대로 폴백 모드, GroundCapable 의 낮거나 기운 자세는 Passive. 로그가 실제 처분을 말하게
                    // 먼저 구한다. ⚠ 래치 중 조작자가 GroundCapable 모드를 누른 그 틱이면 그 모드의 행으로
                    // 판정된다(z_fk < 0.65 ∨ 기울기 ≥ 30° 면 Passive, 아니면 같은 틱에 폴백 모드로 되돌린다) —
                    // spec §4.7 대로 «그 순간의 모드» 기준. 래치 중엔 폴백 모드로 서 있으므로 드물다.
                    const bool to_passive = g1::safety::qd_warn_action(g_mode.row().safety, g_z_fk, g_tilt.value())
                                            == g1::safety::QdWarnAction::Passive;
                    // 래치 에지에서 로그 1줄 + CSV 1건. 해제 뒤라 폴백 모드에서는 같은 틱에 풀려(=no-op) 스팸이 안 난다.
                    if (!warn_before) {
                        if (to_passive) {
                            spdlog::error("[safety] qd_warn LATCHED  |qd|={:.2f} rad/s @ {} {} (warn {:.1f} 을 {}틱 연속 초과)"
                                          "  mode{} z_fk={:.2f} 기울기={:.0f}° -> Passive. 복귀=p→f→m",
                                          qd_now, qd_j, jname(qd_j), js_qd_warn_, js_over_ticks_,
                                          g_mode.row().id, g_z_fk, g_tilt.value());
                            safety_log_.event("qd_warn", qd_j, jname(qd_j), qd_now, js_qd_warn_, "-> Passive. 복귀=p→f→m");
                        } else {
                            spdlog::warn("[safety] qd_warn LATCHED  |qd|={:.2f} rad/s @ {} {} (warn {:.1f} 을 {}틱 연속 초과) -> 폴백 모드 강제. 복귀=키 '1'",
                                         qd_now, qd_j, jname(qd_j), js_qd_warn_, js_over_ticks_);
                            safety_log_.event("qd_warn", qd_j, jname(qd_j), qd_now, js_qd_warn_, "-> 폴백 모드 강제");
                        }
                    }
                    if (to_passive) {
                        // 래치 에지가 아닌데 처분이 Passive 가 된 경우(래치 중 모드가 바뀐 틱)만 따로 한 번 남긴다.
                        if (!js_lowpose_passive_.exchange(true) && warn_before) {
                            spdlog::error("[safety] qd_warn 래치 중 (mode{} z_fk={:.2f} 기울기={:.0f}°) -> Passive. 복귀=p→f→m",
                                          g_mode.row().id, g_z_fk, g_tilt.value());
                            safety_log_.event("qd_warn_lowpose", qd_j, jname(qd_j), qd_now, js_qd_warn_, "-> Passive. 복귀=p→f→m");
                        }
                    } else {
                        g_mode.force(G1_FALLBACK_MODE);
                    }
                }
                // crit은 아래 registered_check가 Passive로 전이시킴(여기선 latch만).
            }
            const auto d_t_safe = clock::now();
            // Controller: detect a mode switch (spline base_vel + mode1 arm-blend), then advance
            // one step so the obs (base_vel_command / ref_foot_height) see fresh values.
            // consume_switch() 는 «직전 consume 때의 모드와 지금» 을 견준다 — 조작 채널이 요청한
            // 것을 같은 틱에 위 qd 가드가 되돌렸으면 전환이 아니다(ModeRuntime.h).
            if (g_mode.consume_switch()) {
                const mode_table::Row& M = g_mode.row();
                g_loco.notify_mode_switch(M.id);
                // mode5 에 들어오면 진입 자세(직립)로 시작한다(spec §4.4) — 사람이 버튼을 누르기 전에도
                // 학습 문법 그대로의 명령이 나간다. 나가면 상태를 비운다(다음 진입은 새로 시작).
                if (M.mode5_cmd_live) {
                    g_m5.press(m5::ENTER_PRESET);
                    printf("\r\n[m5] 진입 → «%s» (자세 키: %s)\r\n", m5::PRESETS[m5::ENTER_PRESET].name, g_m5_key_hint());
                    fflush(stdout);
                } else {
                    g_m5.reset();
                }
                // 되감는 클립: «들어올 때마다» frame 0 으로. 아래 재앵커가 active_ref_loader()
                // 의 «현재 프레임» 자세를 기준으로 init_quat 을 잡으므로, 되감기는 반드시
                // 그보다 먼저다 — 순서가 바뀌면 중간 프레임 heading 에 정렬된 채로 첫
                // 동작이 재생돼 진입 순간 몸이 돈다.
                if (g_ref_is_clip()) if (ClipSlot* c = active_clip()) if (c->rewind_on_enter) {
                    c->t0 = env->episode_length * env->step_dt;
                    c->loader->reset(env->robot->data, 0.0f);
                }
                // RE-ANCHOR at mode change: pin the reference heading to the robot's CURRENT heading
                // so a referenced motion (clip playback / live VR) STARTS from where the robot faces now
                // — no heading snap/turn before it begins. The motion's OWN internal turning is kept
                // (only the start is re-aligned). init_quat was otherwise frozen at FSM enter, which
                // made the robot jump to the enter-heading. (참조가 없는 모드·VR 없는 standby 는 init_quat 을 안 쓴다.)
                if (motion && (g_ref_is_clip() || motion->vr_override)) {
                    auto dl = active_ref_loader();   // 클립 모드면 고른 클립, 아니면 주 로더
                    Eigen::Quaternionf ref_now = g_ref_is_clip() ? dl->root_quaternion_clip()
                                                                 : dl->root_quaternion_vr();
                    // ⚠ auto 금지(이 파일 위쪽 obs 항의 주석과 같은 이유) — 구체 타입으로 받는다.
                    const Eigen::Matrix3f ry = isaaclab::yawQuaternion(ref_now).toRotationMatrix();
                    const Eigen::Matrix3f rr = isaaclab::yawQuaternion(robot_quat_w(env.get())).toRotationMatrix();
                    init_quat = rr * ry.transpose();
                }
                // 🔬 전환 시점의 «레퍼런스 점프» 를 남긴다. 다리 q_ref 는 1초 램프를 타지만
                //    발-z 는(ref 모드) 즉시 점프한다 — 그 짝이 안 맞는지 여기서 바로 보인다.
                if (M.track_lower && motion) {
                    auto dl = active_ref_loader();
                    float qmax = 0.0f; int qj = -1;
                    if (g_ref_is_clip()) {
                        auto qc = dl->joint_pos_clip();
                        for (int i = 0; i < (int)qc.size() && i < 29; ++i) {
                            const float dv = std::fabs(qc[i] - env->robot->data.joint_pos[i]);
                            if (dv > qmax) { qmax = dv; qj = i; }
                        }
                    }
                    const auto fz = (g_ref_is_clip() && dl->has_foot_z) ? dl->foot_z_clip()
                                                                        : g_loco.foot_z;
                    spdlog::info("[diag:switch] -> mode{} ({})  clip frame={}  q_ref 점프 max={:.2f} rad @{} {}"
                                 "  발-z 목표=({:.3f},{:.3f})  생성기=({:.3f},{:.3f})  원천={}",
                                 M.id, M.name, dl ? dl->frame : -1, qmax, qj, qj >= 0 ? jname(qj) : "-",
                                 fz[0], fz[1], g_loco.foot_z[0], g_loco.foot_z[1], g_footz_src_name);
                }
            }
            // Low-pass the base_vel target (deploy-side; controller unchanged) so abrupt
            // GUI/keyboard command changes don't jerk the gait. ~A=0.25 -> ~0.2s settle.
            {
                static std::array<float, 3> bv_s = {0.f, 0.f, 0.f};
                const float A = 0.25f;
                auto bv = g_joystick_base_vel(env.get());
                for (int i = 0; i < 3; ++i) bv_s[i] += A * (bv[i] - bv_s[i]);
                g_loco.update(bv_s, g_mode.mode());
            }
            // 클립 시계: 0번 칸 = 주 클립(구간 오프셋), 그 뒤 = 자기 원점(되감는 칸은 진입 시각이 원점).
            {
                const float t_ep = env->episode_length * env->step_dt;
                for (size_t i = 0; i < g_clips.size(); ++i) {
                    ClipSlot& c = g_clips[i];
                    c.loader->update(i == 0 ? t_ep + time_range_[0] : t_ep - c.t0);
                }
            }
            // 계약 v2: 관측 항이 읽을 이 틱의 mode5 명령·미리보기 (클립 시계가 이 틱으로 옮겨진 뒤).
            {
                static bool m5_hold_prev = false;
                if (g_mode.row().mode5_cmd_live && g_m5.active()) {
                    const auto& gb = env->robot->data.projected_gravity_b;
                    g_m5_cmd = g_m5.tick(g_z_fk, {gb[0], gb[1], gb[2]});
                    if (g_m5.holding() && !m5_hold_prev)
                        spdlog::info("[m5] 도착 «{}»  z_fk={:.2f}  기울기={:.0f}°",
                                     m5::PRESETS[g_m5.preset()].name, g_z_fk, g_tilt.value());
                } else {
                    g_m5_cmd.fill(0.f);
                }
                m5_hold_prev = g_m5.holding();
                // 미리보기는 슬롯이 그 항을 선언했을 때만 (g_obs_has_preview — 계약 v1 슬롯은 계산조차 안 한다).
                // has_preview 는 생성자가 이미 요구했다 — 여기서 다시 보는 것은 lin()/ang() 의 빈 배열 인덱싱을 막는 보루.
                ClipSlot* c = g_ref_is_clip() ? active_clip() : nullptr;
                if (g_obs_has_preview && g_mode.row().motion_preview && c && c->loader->has_preview)
                    g1::preview::block(*c->loader, c->loader->frame, true, g_motion_block);
                else
                    std::fill(g_motion_block.begin(), g_motion_block.end(), 0.f);
            }
            const auto d_t_ctrl = clock::now();
            env->step();
            const auto d_t1 = clock::now();

            // ── 계측 (틱당 ~1 ns. 구간 셋은 env->step() 이 채워 둔 값을 읽기만 한다)
            {
                auto ms = [](clock::time_point a, clock::time_point b) {
                    return std::chrono::duration<double, std::milli>(b - a).count();
                };
                diag.seg(0, ms(d_t0, d_t_poll));
                diag.seg(1, ms(d_t_poll, d_t_safe));
                diag.seg(2, ms(d_t_safe, d_t_ctrl));
                diag.seg(3, env->last_obs_us * 1e-3);
                diag.seg(4, env->last_ort_us * 1e-3);
                diag.seg(5, env->last_act_us * 1e-3);
                diag.tick(ms(d_t0, d_t1), d_first ? 0.0 : ms(d_prev, d_t0));
                d_prev = d_t0; d_first = false;
                if (diag.window_closed()) {
                    diag.format(d_buf, sizeof d_buf, DIAG_SEG);
                    // overrun 이 있으면 경고로 올린다 — 50 Hz 를 못 지킨 것이고, 조용히 지나가면
                    // 나중에 「로봇이 이상하다」로만 보인다.
                    if (diag.overrun() > 0) spdlog::warn("[diag:policy] {}", d_buf + 7);
                    else                    spdlog::info("[diag:policy] {}", d_buf + 7);
                    // 안전층 누적값을 1 Hz 로 시간축에 편다. I/O 는 «여기(50Hz)» 에서만 —
                    // 1 kHz 안전 루프는 이 파일을 건드리지 않는다.
                    safety_log_.sample(mon_clamp_ticks_, mon_clamp_max_, mon_clamp_joint_,
                                       mon_rate_ticks_, mon_rate_max_, mon_rate_joint_,
                                       mon_tilt_max_deg_, &jname);
                    if (d_csv) {
                        char c[512];
                        diag.format_csv(c, sizeof c);
                        std::fprintf(d_csv, "%s\n", c);
                        std::fflush(d_csv);
                    }
                    diag.reset();
                }
            }

            // Sleep
            std::this_thread::sleep_until(sleepTill);
            sleepTill += dt;
        }
        load_run = false;
        if (load_th.joinable()) load_th.join();
        if (d_csv) std::fclose(d_csv);
        safety_log_.close();
    });
}


void State_Mimic::run()
{
    auto action = env->action_manager->processed_actions();   // = raw*scale + offset (target q)
    // mode1 arm-blend: ease the upper-joint TARGETS toward the default pose, then release.
    // Interpolate toward default_joint_pos (= the action offset; joint_names=[.*] so action
    // index == robot joint index). arm_scale is 1.0 except during a mode1 transition.
    g_loco.apply_arm_blend(action.data(), env->robot->data.default_joint_pos.data(),
                           static_cast<int>(action.size()));
    // mode-switch crossfade (+ NaN guard): ease the action from the frozen pre-switch pose to the
    // live action over the blend window so a mode change never sends a raw joint STEP to the
    // motors (which trips the onboard velocity/torque protection). No-op outside a switch.
    g_loco.apply_switch_blend(action.data(), static_cast<int>(action.size()));
    // L1-2차: C++ 최종 위치 clamp (기계한계, gated). action.clip(process_actions)이 1차 방어이고
    // 이건 최종 출력 직전 보루 — 기본 off, sim2sim 검증 후 enable_pos_clamp:true로 켠다.
    // 모니터링 스냅샷 — 안전층이 실제로 값을 깎았는지 전/후 비교로 집계한다. 여기는 1kHz FSM
    // 스레드이므로 절대 I/O 를 하지 않는다(세기만; 출력은 exit() 요약). 29 float 복사는 무시할 비용.
    const int mon_n = static_cast<int>(action.size());
    std::array<float, 29> mon_pre{};
    const bool mon_on = (mon_n <= 29);
    if (mon_on) for (int i = 0; i < mon_n; ++i) mon_pre[i] = action[i];

    if (js_enable_pos_clamp_)
        js_clamp_position(action.data(), js_pos_lo_.data(), js_pos_hi_.data(),
                          static_cast<int>(action.size()));
    if (mon_on && js_enable_pos_clamp_) {
        mon_track(mon_pre.data(), action.data(), mon_n, mon_clamp_ticks_, mon_clamp_max_, mon_clamp_joint_);
        for (int i = 0; i < mon_n; ++i) mon_pre[i] = action[i];   // rate-limit 비교용 기준 갱신
    }
    // 출력 NaN/Inf 홀드 (항상 ON, 2026-09-09). L2 를 끄면서 js_rate_limit 이 겸하던 «NaN→이전값» 을 분리했다.
    // q_prev 는 enter() 에서 측정 pose 로 초기화되고 아래에서 매 틱 갱신되므로 체류 첫 틱부터 유효하다.
    // 1 kHz 스레드 — I/O 없이 세기만 한다(요약은 exit()).
    if (js_q_prev_valid_) {
        const int held = js_hold_nonfinite(action.data(), js_q_prev_.data(), static_cast<int>(action.size()));
        if (held > 0) ++mon_nan_hold_ticks_;
    }
    // L2: 관절 속도 rate-limit (gated, 기본 off). 출력 전용 — obs(last_action 등)는 raw action을
    // 그대로 읽으므로 parity 영향 없음. 비활성일 때도 q_prev는 계속 추적해서, 나중에 켤 때
    // stale q_prev로 인한 첫 틱 점프가 나지 않게 한다.
    if (js_enable_rate_limit_ && js_q_prev_valid_)
        js_rate_limit(action.data(), js_q_prev_.data(), js_max_step_.data(),
                      static_cast<int>(action.size()));
    else
        for (int i = 0; i < (int)action.size(); ++i) js_q_prev_[i] = action[i];
    if (mon_on && js_enable_rate_limit_ && js_q_prev_valid_)
        mon_track(mon_pre.data(), action.data(), mon_n, mon_rate_ticks_, mon_rate_max_, mon_rate_joint_);
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
    // 계측(기본 꺼짐). 명령을 다 실은 «뒤» 라 이 줄의 q_des 는 실제로 나가는 값과 같다.
    if (state_dump_.on()) {
        GaitAux aux = g_loco.probe(g_mode.mode());
        // 보정 «후» 중력 — 정책이 실제로 읽는 그 값. 재계산 아님, 그대로 나른다.
        const auto& pg = env->robot->data.projected_gravity_b;
        aux.pg_x = pg[0]; aux.pg_y = pg[1]; aux.pg_z = pg[2];
        state_dump_.tick(FSMState::lowstate->msg_, lowcmd->msg_, aux);
    }
}

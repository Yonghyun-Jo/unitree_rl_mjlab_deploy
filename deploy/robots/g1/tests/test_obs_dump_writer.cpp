// test_obs_dump_writer.cpp — ObsDump(G1_OBS_CSV, P1 대조 계측)의 «쓰기 경로» 와 «칸이 맞는가» 를 잠근다.
//
// 여기서 재는 것:
//   ⓪ 계측기 자기검증 — 이 스레드의 write/read syscall 카운터(/proc/thread-self/io)와 할당 카운터가 실제로 움직이는가
//   ① 헤더 = # 머리 + 열 이름(고정 285 칸 + obs), 모든 줄의 열 수가 같다
//   ② 칸마다 값이 그대로 나르는가 — 기대값은 «멤버 이름으로» 따로 적은 표(expect_col)에서 온다. 이름표(fields())와
//      독립이라 fields() 가 칸을 바꿔 적거나(kp ↔ kd) 빠뜨리면 여기서 걸린다
//   ③ 변이 시험 — 기록 칸 하나를 바꾼 덤프는 «정확히 그 줄·그 칸» 하나만 어긋난다고 나와야 한다(검사가 실제로 잡는다)
//   ④ 자극이 움직이는가 — 칸마다 줄 사이에 값이 바뀐다(멈춘 기록기가 «같은 값 == 같은 값» 으로 통과하지 못하게)
//   ⑤ 🔴 RT 경로(begin_row·stage·publish_step·record_out)가 syscall 0 · 할당 0
//   ⑥ run() 기록(시퀀스 잠금)이 동시에 쓰여도 찢긴 기록이 줄에 들어가지 않는다
//   ⑦ 멱등 open · obs 배치가 다른 두 번째 open 은 false · shared() 하나 · close 여러 번 · 꺼짐(환경변수 없음) = no-op
//
//   g++ -std=gnu++17 -O3 -DNDEBUG -Wall -Wextra -I../include -pthread test_obs_dump_writer.cpp -o /tmp/t && /tmp/t
#include "ObsDump.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <set>
#include <string>
#include <thread>
#include <vector>

static int fail = 0;
static void chk(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL %s\n", what); ++fail; }
}

// ── 할당 카운터: 이 스레드가 켠 동안의 operator new 호출 수 ─────────────────────────────────────
// 전역 new/delete 를 malloc/free 로 바꿔 끼운다 — g++ 11+ 는 그 짝을 «맞지 않는 할당» 으로 오인해 경고한다(거짓 경보).
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 11
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif
static thread_local bool g_count_alloc = false;
static std::atomic<long> g_allocs{0};
void* operator new(std::size_t n) {
    if (g_count_alloc) g_allocs.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

// 이 스레드의 write·read 계열 syscall 누적 수(/proc/thread-self/io 의 syscw·syscr). 못 읽으면 −1.
// 🔴 «/proc/self/syscw» 같은 파일은 없다 — 필드다(StateDump 시험이 −1 − (−1) = 0 으로 통과하는 척한 사고).
static void io_pair(long& w, long& r) {
    w = r = -1;
    std::FILE* f = std::fopen("/proc/thread-self/io", "r");
    if (!f) return;
    char k[64]; long v = 0;
    while (std::fscanf(f, "%63s %ld", k, &v) == 2) {
        if (std::strcmp(k, "syscr:") == 0) r = v;
        if (std::strcmp(k, "syscw:") == 0) w = v;
    }
    std::fclose(f);
}
// 재는 일 자체(fopen·fscanf)가 쓰는 syscall 을 뺀다 — 빈 구간을 재서 그만큼을 기준선으로.
struct IoMeter {
    long bw = 0, br = 0, w0 = -1, r0 = -1;
    IoMeter() { long a, b, c, d; io_pair(a, b); io_pair(c, d); bw = c - a; br = d - b; }
    void start() { io_pair(w0, r0); }
    // 반환: 이 구간의 (write, read) — 못 읽으면 큰 값(판정이 «통과하는 척» 하지 않게)
    void stop(long& w, long& r) const {
        long w1, r1; io_pair(w1, r1);
        const bool ok = w0 >= 0 && r0 >= 0 && w1 >= 0 && r1 >= 0;
        w = ok ? (w1 - w0) - bw : 1000000;
        r = ok ? (r1 - r0) - br : 1000000;
    }
};

static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    size_t a = 0;
    for (;;) {
        const size_t b = s.find(',', a);
        out.push_back(s.substr(a, b == std::string::npos ? std::string::npos : b - a));
        if (b == std::string::npos) break;
        a = b + 1;
    }
    return out;
}
struct Csv { std::vector<std::string> meta, cols; std::vector<std::vector<std::string>> rows; };
static Csv read_csv(const char* path) {
    Csv c;
    std::FILE* f = std::fopen(path, "r");
    if (!f) return c;
    std::string line;
    int ch;
    auto flush_line = [&] {
        if (line.empty()) return;
        if (line[0] == '#') c.meta.push_back(line);
        else if (c.cols.empty()) c.cols = split(line);
        else c.rows.push_back(split(line));
        line.clear();
    };
    while ((ch = std::fgetc(f)) != EOF) {
        if (ch == '\n') flush_line(); else line.push_back(char(ch));
    }
    flush_line();
    std::fclose(f);
    return c;
}

// ── 자극: 칸마다 다르고 줄마다 바뀌는 값 (멤버 «이름» 으로 채운다) ─────────────────────────────────
static constexpr int NJ = 29;
static constexpr int N_OBS = 3006;   // v7 슬롯 obs 폭 — 실제 줄 길이로 돈다
static float fv(int k, int field, int i) { return 0.123456789f * float(field + 1) + 0.0137f * float(i) + 0.001953125f * float(k); }
static void fill_row(g1::ObsDumpRow& r, int k) {
    r.episode = 1000 + k;
    r.key = 'a' + k % 26;   r.mode = 1 + k % 6;      r.mode_req = 6 - k % 6;
    r.switched = k % 4;     r.clip = k % 3;          r.preset = k % 9;
    r.m5_hold = k % 2;      r.qd_warn = (k / 3) % 2; r.vr = (k / 5) % 2;
    r.gui_seq = 7000 + 3 * k;
    for (int i = 0; i < 3; ++i) {
        r.kb[i] = fv(k, 1, i); r.bv[i] = -fv(k, 2, i); r.bvf[i] = fv(k, 3, i); r.base_vel[i] = fv(k, 4, i);
        r.g1[i] = fv(k, 5, i); r.gyro[i] = fv(k, 6, i); r.pg[i] = -fv(k, 7, i);
    }
    r.z_fk = fv(k, 8, 0); r.tilt = fv(k, 9, 0); r.tilt_raw = fv(k, 10, 0);
    for (int i = 0; i < 4; ++i) { r.quat1[i] = fv(k, 11, i); r.init_quat[i] = fv(k, 12, i); r.quat[i] = fv(k, 13, i); }
    for (int j = 0; j < NJ; ++j) {
        r.q1[j] = fv(k, 14, j); r.q[j] = fv(k, 15, j); r.dq[j] = -fv(k, 16, j); r.raw[j] = fv(k, 17, j);
    }
}
static float obs_v(int k, int i) { return float(k) * 0.5f + 1e-4f * float(i) + 0.25f; }
// run() 기록은 «어느 줄의 action 이었나(from)» 로 정해진다 — 찢긴 기록을 알아볼 수 있게
static float out_v(long long from, int which, int i) { return float(from) * 0.03125f + 0.5f * float(which) + 0.001f * float(i); }
static void run_record(g1::ObsDump& d, long long from) {
    float p[NJ], q[NJ], kp[NJ], kd[NJ];
    for (int i = 0; i < NJ; ++i) { p[i] = out_v(from, 0, i); q[i] = out_v(from, 1, i); kp[i] = out_v(from, 2, i); kd[i] = out_v(from, 3, i); }
    d.record_out(from, p, q, kp, kd);
}

// 열 이름 → 기대값. fields() 를 거치지 않는다(멤버를 이름으로 직접) — 이름표가 틀리면 여기서 갈린다.
static bool expect_col(const g1::ObsDumpRow& r, const std::vector<float>& obs, const std::string& c, double& v) {
    auto pre = [&](const char* p, int& i) {
        const size_t n = std::strlen(p);
        if (c.size() <= n || c.compare(0, n, p) != 0) return false;
        i = std::atoi(c.c_str() + n);
        return true;
    };
    int i = 0;
    if (c == "row") v = double(r.row);           else if (c == "kind") v = r.kind;
    else if (c == "stay") v = r.stay;            else if (c == "episode") v = double(r.episode);
    else if (c == "key") v = r.key;              else if (c == "mode") v = r.mode;
    else if (c == "mode_req") v = r.mode_req;    else if (c == "switched") v = r.switched;
    else if (c == "clip") v = r.clip;            else if (c == "preset") v = r.preset;
    else if (c == "m5_hold") v = r.m5_hold;      else if (c == "qd_warn") v = r.qd_warn;
    else if (c == "vr") v = r.vr;                else if (c == "gui_seq") v = r.gui_seq;
    else if (c == "z_fk") v = r.z_fk;            else if (c == "tilt") v = r.tilt;
    else if (c == "tilt_raw") v = r.tilt_raw;    else if (c == "out_from") v = double(r.out_from);
    else if (pre("kb_", i)) v = r.kb[i];         else if (pre("bvf_", i)) v = r.bvf[i];
    else if (pre("bv_", i)) v = r.bv[i];         else if (pre("base_vel_", i)) v = r.base_vel[i];
    else if (pre("g1_", i)) v = r.g1[i];         else if (pre("quat1_", i)) v = r.quat1[i];
    else if (pre("q1_", i)) v = r.q1[i];         else if (pre("init_quat_", i)) v = r.init_quat[i];
    else if (pre("q_des_", i)) v = r.q_des[i];   else if (pre("q_", i)) v = r.q[i];
    else if (pre("dq_", i)) v = r.dq[i];         else if (pre("quat_", i)) v = r.quat[i];
    else if (pre("gyro_", i)) v = r.gyro[i];     else if (pre("pg_", i)) v = r.pg[i];
    else if (pre("raw_", i)) v = r.raw[i];       else if (pre("proc_", i)) v = r.proc[i];
    else if (pre("kp_", i)) v = r.kp[i];         else if (pre("kd_", i)) v = r.kd[i];
    else if (pre("obs_", i)) v = obs[size_t(i)];
    else return false;
    return true;
}

// 덤프 하나를 쓴다: 체류 줄 1 + 틱 줄 n. mutate(k, row) 가 있으면 «기록한 뒤» 칸 하나를 바꾼다(변이).
// expected 에는 변이 «전» 값을 담는다 — 검사기가 그 차이를 찾아야 한다.
struct Want { g1::ObsDumpRow r; std::vector<float> obs; };
template <class Mut>
static std::vector<Want> write_dump(g1::ObsDump& d, int n, Mut mutate, long* sysw_rt, long* sysr_rt, long* allocs_rt) {
    std::vector<Want> want;
    want.reserve(size_t(n) + 1);
    Want w;
    const int no = d.n_obs();   // 이 덤프의 obs 폭(⑦ 은 8)
    w.obs.assign(size_t(no), 0.f);
    IoMeter io;
    long sw = 0, sr = 0, al = 0;
    d.mark_enter();
    for (int k = 0; k <= n; ++k) {
        // 기대값 먼저(할당이 있으니 계측 밖에서)
        w.r.clear();
        fill_row(w.r, k);
        for (int i = 0; i < no; ++i) w.obs[size_t(i)] = obs_v(k, i);
        io.start();
        g_allocs.store(0); g_count_alloc = true;
        // ── RT 경로(정책 스레드가 하는 일) ──
        d.begin_row(k == 0 ? 0 : 1);
        g1::ObsDumpRow& s = d.stage();
        const int64_t row = s.row; const int32_t kind = s.kind, stay = s.stay; const double te = s.t_enter;
        fill_row(s, k);
        std::memcpy(d.stage_obs(), w.obs.data(), sizeof(float) * size_t(no));
        d.publish_step();
        // ── run() 이 한 일(이 줄의 action 을 실었다)
        run_record(d, d.step_seq());
        mutate(k, s);
        g_count_alloc = false; al += g_allocs.load();
        long dw, dr; io.stop(dw, dr);
        sw += dw; sr += dr;
        w.r.row = row; w.r.kind = kind; w.r.stay = stay; w.r.t_enter = te;
        for (int i = 0; i < NJ; ++i) {
            w.r.proc[i] = out_v(row, 0, i); w.r.q_des[i] = out_v(row, 1, i);
            w.r.kp[i] = out_v(row, 2, i);   w.r.kd[i] = out_v(row, 3, i);
        }
        w.r.out_from = row;
        want.push_back(w);
        std::this_thread::sleep_for(std::chrono::microseconds(500));   // 쓰기 스레드가 따라오게(링 512 줄)
    }
    d.flush_pending();
    if (sysw_rt) *sysw_rt = sw;
    if (sysr_rt) *sysr_rt = sr;
    if (allocs_rt) *allocs_rt = al;
    return want;
}
static std::vector<Want> write_dump(g1::ObsDump& d, int n) {
    return write_dump(d, n, [](int, g1::ObsDumpRow&) {}, nullptr, nullptr, nullptr);
}

// 덤프 ↔ 기대값. 어긋난 (줄, 칸) 목록을 돌려준다. t 는 증가만, t_enter 는 체류 줄만 유한.
struct Miss { int row; std::string col; };
static std::vector<Miss> verify(const Csv& c, const std::vector<Want>& want, int* handled_cols) {
    std::vector<Miss> miss;
    int handled = 0;
    for (const auto& col : c.cols) {
        double v;
        if (col == "t" || col == "t_enter" || expect_col(want.at(0).r, want.at(0).obs, col, v)) ++handled;
        else miss.push_back({-1, "모르는 칸 " + col});
    }
    if (handled_cols) *handled_cols = handled;
    if (c.rows.size() != want.size()) { miss.push_back({-1, "줄 수"}); return miss; }
    double t_prev = -1;
    for (size_t k = 0; k < want.size(); ++k) {
        const auto& row = c.rows[k];
        if (row.size() != c.cols.size()) { miss.push_back({int(k), "열 수"}); continue; }
        for (size_t j = 0; j < c.cols.size(); ++j) {
            const std::string& col = c.cols[j];
            const double got = std::strtod(row[j].c_str(), nullptr);
            if (col == "t") { if (!(got >= t_prev)) miss.push_back({int(k), col}); t_prev = got; continue; }
            if (col == "t_enter") { if (std::isfinite(got) != (want[k].r.kind == 0)) miss.push_back({int(k), col}); continue; }
            double v;
            if (!expect_col(want[k].r, want[k].obs, col, v)) continue;
            // %.9g 는 float32 왕복이 정확하다 → float 로 되돌려 «정확히» 같아야 한다(정수 칸은 2^24 아래라 float 로도 정확)
            const bool same = (std::isnan(v) && std::isnan(got)) || got == v || float(got) == float(v);
            if (!same) miss.push_back({int(k), col});
        }
    }
    return miss;
}

static g1::ObsDump::Meta meta_of(const char* state, int n_obs) {
    g1::ObsDump::Meta m;
    m.slot = "test_slot"; m.state = state; m.network = "lo"; m.commit = "deadbeef0000"; m.dirty = 0;
    m.obs_contract = "a:1:1"; m.obs_contract_version = "2"; m.obs_terms = "a:a:1:1";
    m.groups = {{"obs", n_obs}};
    return m;
}

int main() {
    // ⓪ 계측기 자기검증 — 바늘이 안 움직이면 ⑤ 는 아무것도 재지 않는다
    {
        IoMeter io;
        std::FILE* t = std::fopen("/tmp/_todw_probe.txt", "w");
        io.start();
        for (int i = 0; i < 200; ++i) { std::fprintf(t, "x%d\n", i); std::fflush(t); }
        long dw, dr; io.stop(dw, dr);
        std::FILE* t2 = std::fopen("/tmp/_todw_probe.txt", "r");
        char buf[16];
        io.start();
        for (int i = 0; i < 50; ++i) { std::rewind(t2); std::setvbuf(t2, nullptr, _IONBF, 0); (void)!std::fread(buf, 1, 4, t2); }
        long dw2, dr2; io.stop(dw2, dr2);
        std::fclose(t); std::fclose(t2);
        chk(dw >= 200 && dw < 1000000, "이 스레드의 syscw 카운터가 안 움직인다 — ⑤ 판정이 무력하다");
        chk(dr2 >= 50 && dr2 < 1000000, "이 스레드의 syscr 카운터가 안 움직인다 — ⑤ 판정이 무력하다");
        g_allocs.store(0); g_count_alloc = true;
        { std::vector<int>* v = new std::vector<int>(10); delete v; }
        g_count_alloc = false;
        chk(g_allocs.load() >= 2, "할당 카운터가 안 움직인다 — ⑤ 판정이 무력하다");
        std::printf("  ⓪ 계측기 자기검증: fflush 200 -> syscw +%ld · fread 50 -> syscr +%ld · new 2 -> 할당 %ld\n", dw, dr2, g_allocs.load());
        std::remove("/tmp/_todw_probe.txt");
    }

    const int kRows = 400;
    // ①②④⑤ 정상 덤프
    {
        const char* path = "/tmp/_todw.csv";
        std::remove(path);
        setenv("G1_OBS_CSV", path, 1);
        long sysw = 0, sysr = 0, allocs = 0;
        std::vector<Want> want;
        {
            g1::ObsDump d;
            chk(d.open_from_env("G1_OBS_CSV", meta_of("Mimic_Masked", N_OBS)), "열리지 않았다");
            chk(d.on() && d.n_obs() == N_OBS, "on · n_obs");
            want = write_dump(d, kRows, [](int, g1::ObsDumpRow&) {}, &sysw, &sysr, &allocs);
            chk(d.dropped() == 0, "평시에 줄을 버렸다");
            d.close();
        }
        const Csv c = read_csv(path);
        chk(c.meta.size() == 9 && c.meta[0] == "# g1_obs_dump v1", "# 머리 9 줄");
        bool has_slot = false, has_commit = false;
        for (const auto& m : c.meta) {
            if (m == "# slot=test_slot") has_slot = true;
            if (m == "# commit=deadbeef0000 dirty=0") has_commit = true;
        }
        chk(has_slot && has_commit, "머리에 슬롯 · 커밋");
        // 고정 285 칸(손으로 센 값 — 칸이 늘거나 줄면 분석기가 조용히 어긋나므로 시험이 먼저 알린다) + obs
        chk(int(c.cols.size()) == 285 + N_OBS, "열 수 = 285 + obs");
        if (int(c.cols.size()) != 285 + N_OBS) std::printf("     열 %zu, 기대 %d\n", c.cols.size(), 285 + N_OBS);
        int handled = 0;
        const auto miss = verify(c, want, &handled);
        chk(handled == int(c.cols.size()), "모든 칸을 기대값 표가 안다");
        chk(miss.empty(), "② 값이 그대로 나르지 않는다");
        for (size_t i = 0; i < miss.size() && i < 8; ++i) std::printf("     어긋남 줄 %d 칸 %s\n", miss[i].row, miss[i].col.c_str());
        // ④ 자극이 움직이는가 — 틱 줄 사이에 칸마다 값이 둘 이상(체류 번호·종류·t_enter 만 상수)
        if (c.rows.size() == want.size()) {
            int still = 0;
            for (size_t j = 0; j < c.cols.size(); ++j) {
                const std::string& col = c.cols[j];
                if (col == "kind" || col == "stay" || col == "t_enter") continue;
                std::set<std::string> vals;
                for (size_t k = 1; k < c.rows.size() && vals.size() < 2; ++k) vals.insert(c.rows[k][j]);
                if (vals.size() < 2) { if (still < 5) std::printf("     안 움직이는 칸 %s\n", col.c_str()); ++still; }
            }
            chk(still == 0, "④ 자극이 안 움직이는 칸이 있다 — 멈춘 기록기를 못 잡는다");
        }
        // ⑤ RT 경로
        std::printf("  ⑤ RT 경로 %d 줄: write syscall %ld · read syscall %ld · 할당 %ld\n", kRows + 1, sysw, sysr, allocs);
        chk(sysw == 0, "⑤ RT 경로가 write 계열 syscall 을 했다 = 파일을 만진다");
        chk(sysr == 0, "⑤ RT 경로가 read 계열 syscall 을 했다");
        chk(allocs == 0, "⑤ RT 경로가 할당했다");
        std::printf("  ①② %zu 줄 × %zu 칸 — 칸마다 값 일치 · ④ 자극 움직임\n", c.rows.size(), c.cols.size());
    }

    // ③ 변이 시험 — 기록 칸 하나를 바꾸면 정확히 그 칸 하나만 어긋나야 한다
    {
        struct Case { const char* col; int k; };
        const Case cases[] = {{"kd_5", 37}, {"mode", 120}, {"obs_2999", 200}, {"q1_0", 1}, {"init_quat_3", 399}};
        for (const auto& cs : cases) {
            const char* path = "/tmp/_todw_mut.csv";
            std::remove(path);
            setenv("G1_OBS_CSV", path, 1);
            std::vector<Want> want;
            {
                g1::ObsDump d;
                d.open_from_env("G1_OBS_CSV", meta_of("Mimic_Masked", N_OBS));
                const std::string col = cs.col;
                const int kk = cs.k;
                want = write_dump(d, kRows, [&](int k, g1::ObsDumpRow& s) {
                    if (k != kk) return;
                    if (col == "kd_5") s.kd[5] += 1.f;          // ⚠ record_out 뒤라 이 칸은 push 때 run() 기록이 덮는다 → 아래 따로
                    else if (col == "mode") s.mode += 1;
                    else if (col == "obs_2999") d.stage_obs()[2999] += 1.f;
                    else if (col == "q1_0") s.q1[0] += 1e-6f;
                    else if (col == "init_quat_3") s.init_quat[3] = -s.init_quat[3];
                }, nullptr, nullptr, nullptr);
                // kd 는 run() 쪽 기록이다 — 그 줄의 기록을 다시 써서(찢김 없이) 바꾼다: 다음 begin_row 전에 한 번 더
                d.close();
            }
            const Csv c = read_csv(path);
            const auto miss = verify(c, want, nullptr);
            const bool exact = miss.size() == 1 && miss[0].row == cs.k && miss[0].col == cs.col;
            if (std::string(cs.col) == "kd_5") {
                // kd 는 run() 의 기록 칸이라 정책 스레드가 stage 에 쓴 값은 push 때 덮인다(기록 원천이 하나라는 것의 증거)
                chk(miss.empty(), "③ run() 기록 칸(kd)을 정책 스레드가 덮어쓸 수 있으면 안 된다");
                std::printf("  ③ 변이 %-12s 줄 %3d → 덮임(run() 기록이 원천) %s\n", cs.col, cs.k, miss.empty() ? "✓" : "✗");
            } else {
                chk(exact, "③ 변이를 정확히 그 줄·칸 하나로 잡지 못했다");
                std::printf("  ③ 변이 %-12s 줄 %3d → 어긋남 %zu 건%s\n", cs.col, cs.k, miss.size(),
                            exact ? " (정확히 그 칸)" : "");
                for (size_t i = 0; i < miss.size() && i < 4; ++i) std::printf("       줄 %d 칸 %s\n", miss[i].row, miss[i].col.c_str());
            }
        }
        // run() 기록 칸의 변이: run() 이 «다른 값» 을 실으면(기록이 틀리면) 그 줄의 q_des 가 어긋나야 한다
        {
            const char* path = "/tmp/_todw_mut2.csv";
            std::remove(path);
            setenv("G1_OBS_CSV", path, 1);
            std::vector<Want> want;
            {
                g1::ObsDump d;
                d.open_from_env("G1_OBS_CSV", meta_of("Mimic_Masked", N_OBS));
                want = write_dump(d, 60, [&](int k, g1::ObsDumpRow&) {
                    if (k != 17) return;
                    float p[NJ], q[NJ], kp[NJ], kd[NJ];
                    const long long from = d.step_seq();
                    for (int i = 0; i < NJ; ++i) { p[i] = out_v(from, 0, i); q[i] = out_v(from, 1, i); kp[i] = out_v(from, 2, i); kd[i] = out_v(from, 3, i); }
                    q[11] += 2e-6f;
                    d.record_out(from, p, q, kp, kd);
                }, nullptr, nullptr, nullptr);
                d.close();
            }
            const auto miss = verify(read_csv(path), want, nullptr);
            const bool exact = miss.size() == 1 && miss[0].row == 17 && miss[0].col == "q_des_11";
            chk(exact, "③ run() 이 실은 q_des 하나가 틀리면 그 칸 하나로 잡혀야 한다");
            std::printf("  ③ 변이 q_des_11(run) 줄  17 → 어긋남 %zu 건%s\n", miss.size(), exact ? " (정확히 그 칸)" : "");
        }
    }

    // ⑥ 시퀀스 잠금 — run() 이 촘촘히 덮어써도 줄에 찢긴 기록이 안 들어간다. 기록마다 값이 다르게(쓰기 횟수 c 를
    //    모든 칸에 섞는다) — 같은 값을 거듭 쓰면 찢겨도 안 보인다(바늘이 안 움직이는 시험).
    {
        const char* path = "/tmp/_todw_seq.csv";
        std::remove(path);
        setenv("G1_OBS_CSV", path, 1);
        auto enc = [](long long from, long long c, int which, int i) {
            return float((c * 7 + i * 3 + which * 11 + from * 13) % 1000003);   // 2^24 아래 정수 = float 정확
        };
        const int kSeqRows = 1500;
        int torn = 0, own = 0, n = 0, missed = 0;
        unsigned long long dropped = 0;
        {
            g1::ObsDump d;
            d.open_from_env("G1_OBS_CSV", meta_of("Mimic_Masked", 8));
            std::atomic<bool> go{true};
            // run() 흉내: 쓰고 ~1 µs 쉰다(실제는 1 ms 마다 한 번 — 이보다 천 배 촘촘하게 겹친다). 쉬지 않고 쓰면
            // 시퀀스 잠금 독자는 굶는다(짝수 순간이 없다) — 그건 실제 run() 에 없는 부하다.
            std::thread runner([&] {
                long long c = 0;
                float p[NJ], q[NJ], kp[NJ], kd[NJ];
                while (go.load(std::memory_order_relaxed)) {
                    ++c;
                    const long long from = d.step_seq();
                    for (int i = 0; i < NJ; ++i) { p[i] = enc(from, c, 0, i); q[i] = enc(from, c, 1, i); kp[i] = enc(from, c, 2, i); kd[i] = enc(from, c, 3, i); }
                    p[NJ - 1] = float(c % 1000003);
                    d.record_out(from, p, q, kp, kd);
                    const auto t1 = std::chrono::steady_clock::now() + std::chrono::microseconds(1);
                    while (std::chrono::steady_clock::now() < t1) {}
                }
            });
            d.mark_enter();
            for (int k = 0; k <= kSeqRows; ++k) {
                d.begin_row(k == 0 ? 0 : 1);
                fill_row(d.stage(), k);
                d.publish_step();
                std::this_thread::sleep_for(std::chrono::microseconds(60));
            }
            d.flush_pending();
            go = false;
            runner.join();
            d.close();
            dropped = d.dropped();   // 링(512 줄)보다 빨리 넣는다 — 버린 줄은 세어져야 한다
        }
        const Csv c = read_csv(path);
        std::vector<size_t> iq, ik, iproc;
        size_t i_from = 0, i_row = 0;
        for (size_t j = 0; j < c.cols.size(); ++j) {
            if (c.cols[j].rfind("q_des_", 0) == 0) iq.push_back(j);
            if (c.cols[j].rfind("kd_", 0) == 0) ik.push_back(j);
            if (c.cols[j].rfind("proc_", 0) == 0) iproc.push_back(j);
            if (c.cols[j] == "out_from") i_from = j;
            if (c.cols[j] == "row") i_row = j;
        }
        const bool shape = iq.size() == size_t(NJ) && ik.size() == size_t(NJ) && iproc.size() == size_t(NJ);
        chk(shape, "⑥ 칸 모양");
        for (const auto& r : c.rows) {
            if (!shape) break;
            ++n;
            const long long from = std::atoll(r[i_from].c_str());
            if (from == std::atoll(r[i_row].c_str())) ++own;
            if (from == -2) { ++missed; continue; }   // 못 읽음 = nan 으로 «없음» 표시(찢김 아님)
            const long long cc = (long long)std::strtof(r[iproc[NJ - 1]].c_str(), nullptr);
            bool ok = true;
            for (int i = 0; ok && i < NJ; ++i)
                ok = std::strtof(r[iq[size_t(i)]].c_str(), nullptr) == enc(from, cc, 1, i) &&
                     std::strtof(r[ik[size_t(i)]].c_str(), nullptr) == enc(from, cc, 3, i) &&
                     (i == NJ - 1 || std::strtof(r[iproc[size_t(i)]].c_str(), nullptr) == enc(from, cc, 0, i));
            if (!ok) ++torn;
        }
        chk((unsigned long long)n + dropped == (unsigned long long)kSeqRows + 1, "⑥ 줄 수 + 버린 줄 = 넣은 줄");
        chk(torn == 0, "⑥ 찢긴 run() 기록이 줄에 들어갔다");
        chk(own >= n - 10, "⑥ 줄 대부분이 자기 action 의 기록(out_from == row)이어야 한다");
        chk(missed <= 10, "⑥ 시퀀스 잠금 독자가 너무 자주 굶는다");
        std::printf("  ⑥ 시퀀스 잠금: %d 줄(+버림 %llu) · 찢김 %d · 못 읽음 %d · 자기 기록 %d\n", n, dropped, torn, missed, own);
    }

    // ⑦ 멱등 · 다른 배치 · shared() · 꺼짐
    {
        {   // 꺼짐: 환경변수 없음 → open false, 나머지는 no-op
            unsetenv("G1_OBS_CSV");
            g1::ObsDump off;
            chk(!off.open_from_env("G1_OBS_CSV", meta_of("Mimic_Masked", 8)), "환경변수 없이 열렸다");
            chk(!off.on(), "꺼짐인데 on");
            off.mark_enter(); off.begin_row(1); off.publish_step(); off.flush_pending();
            float z[NJ] = {};
            off.record_out(1, z, z, z, z);
            chk(off.rows_pushed() == 0, "꺼짐인데 줄을 넣었다");
            off.close(); off.close();
        }
        const char* path = "/tmp/_todw_idem.csv";
        const char* other = "/tmp/_todw_idem_other.csv";
        std::remove(path); std::remove(other);
        setenv("G1_OBS_CSV", path, 1);
        {
            g1::ObsDump d;
            chk(d.open_from_env("G1_OBS_CSV", meta_of("Mimic_Masked", 8)), "첫 open");
            write_dump(d, 3);
            chk(d.open_from_env("G1_OBS_CSV", meta_of("Mimic_Masked", 8)), "같은 배치 재오픈은 true(멱등)");
            chk(!d.open_from_env("G1_OBS_CSV", meta_of("Mimic_Dance1_subject2", 9)), "다른 obs 배치는 false");
            setenv("G1_OBS_CSV", other, 1);
            chk(d.open_from_env("G1_OBS_CSV", meta_of("Mimic_Masked", 8)), "다른 경로 요청 = 원래 파일 계속(true)");
            std::FILE* f = std::fopen(other, "r");
            chk(f == nullptr, "다른 경로 요청이 새 파일을 만들었다");
            if (f) std::fclose(f);
            setenv("G1_OBS_CSV", path, 1);
            write_dump(d, 2);   // 두 번째 체류
            d.close(); d.close();
        }
        const Csv c = read_csv(path);
        int heads = 0;
        for (const auto& m : c.meta) if (m == "# g1_obs_dump v1") ++heads;
        chk(heads == 1 && c.rows.size() == size_t(4 + 3), "재오픈 뒤 헤더 1 회 · 두 체류의 줄 보존");
        size_t is = 0;
        for (size_t j = 0; j < c.cols.size(); ++j) if (c.cols[j] == "stay") is = j;
        if (c.rows.size() == 7) chk(c.rows[0][is] == "1" && c.rows[6][is] == "2", "체류 번호 1 → 2");
        chk(&g1::ObsDump::shared() == &g1::ObsDump::shared(), "shared() 는 하나");
        std::printf("  ⑦ 멱등 · 다른 배치 거부 · 꺼짐 no-op · shared()\n");
    }

    std::printf(fail ? "test_obs_dump_writer: %d FAIL\n" : "test_obs_dump_writer: OK\n", fail);
    return fail ? 1 : 0;
}

// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <iostream>
#include <mutex>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaaclab
{

// obs 계약의 항 하나. 배포 쪽(deploy.yaml)이 채워 verify_inputs 에 넘긴다.
struct ObsTermSpec
{
    std::string deploy_name;   // deploy.yaml 항 이름 (= C++ REGISTER_OBSERVATION 이름)
    std::string train_name;    // 학습(mjlab) 쪽 같은 항의 이름. deploy.yaml `train_term:`
    int dim = 0;               // 한 프레임 차원 (history 곱하기 전)
    int history = 1;
};

// ONNX metadata_props["obs_contract"] 형식: "이름:차원:history,이름:차원:history,..."
// 적힌 «순서» 가 곧 obs 배치 순서다. 항 이름은 식별자라 ':' ',' 와 안 부딪힌다.
struct ObsContractTerm { std::string name; int dim = 0; int history = 1; };

inline std::vector<ObsContractTerm> parse_obs_contract(const std::string& spec)
{
    std::vector<ObsContractTerm> out;
    std::stringstream ss(spec);
    std::string item;
    while (std::getline(ss, item, ',')) {
        // 공백 제거 (사람이 손으로 고칠 수도 있으니)
        item.erase(std::remove_if(item.begin(), item.end(),
                                  [](unsigned char c) { return std::isspace(c); }),
                   item.end());
        if (item.empty()) continue;
        const auto a = item.find(':');
        const auto b = item.find(':', a == std::string::npos ? 0 : a + 1);
        if (a == std::string::npos || b == std::string::npos) {
            throw std::runtime_error("obs_contract 형식 오류 (이름:차원:history 여야 한다): '" + item + "'");
        }
        ObsContractTerm t;
        t.name = item.substr(0, a);
        t.dim = std::stoi(item.substr(a + 1, b - a - 1));
        t.history = std::stoi(item.substr(b + 1));
        out.push_back(t);
    }
    return out;
}

class Algorithms
{
public:
    virtual std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) = 0;

    // 기동 시 «obs 계약» 대조. 모델이 요구하는 입력 이름·크기와 실제 obs 를 맞춰 보고
    // 어긋나면 throw 한다. 기본은 no-op — 모델이 자기 입력을 모르는 구현체도 있을 수 있다.
    virtual void verify_inputs(const std::unordered_map<std::string, std::vector<float>>&,
                               const std::vector<ObsTermSpec>& = {}) const {}

    std::vector<float> get_action()
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }
    
    std::vector<float> action;
protected:
    std::mutex act_mtx_;
};

class OrtRunner : public Algorithms
{
public:
    OrtRunner(std::string model_path)
    {
        // Init Model
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        // Deploy control models are tiny (e.g. 1670->29) and run at 50 Hz. ONNX Runtime's default
        // intra-op pool (= all cores) SPIN-WAITS between inferences and burns ~10 cores, starving the
        // sim / other work. Pin to 1 thread + disable spinning -> ~1 core, no latency cost (<1ms/infer).
        session_options.SetIntraOpNumThreads(1);
        session_options.SetInterOpNumThreads(1);
        session_options.AddConfigEntry("session.intra_op.allow_spinning", "0");

        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.push_back(input_name.release());
        }

        for (const auto& shape : input_shapes) {
            size_t size = 1;
            for (const auto& dim : shape) {
                size *= dim;
            }
            input_sizes.push_back(size);
        }

        // Get output shape
        Ort::TypeInfo output_type = session->GetOutputTypeInfo(0);
        output_shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
        auto output_name = session->GetOutputNameAllocated(0, allocator);
        output_names.push_back(output_name.release());

        action.resize(output_shape[1]);
    }

    // 이 모델이 받는 입력을 0 으로 채워 돌려준다. 진단용 부하 주입이 입력 이름/크기를
    // 손으로 적지 않아도 되게 — 손으로 적으면 모델을 바꿀 때마다 조용히 틀린다.
    std::unordered_map<std::string, std::vector<float>> zero_obs() const
    {
        std::unordered_map<std::string, std::vector<float>> o;
        for (size_t i = 0; i < input_names.size(); ++i)
            o[std::string(input_names[i])] = std::vector<float>(input_sizes[i], 0.0f);
        return o;
    }

    // 이 모델이 요구하는 입력 이름 -> 원소 개수. ONNX 가 스스로 아는 값이라 «진실» 이다.
    std::unordered_map<std::string, int64_t> input_sizes_by_name() const
    {
        std::unordered_map<std::string, int64_t> m;
        for (size_t i = 0; i < input_names.size(); ++i)
            m[std::string(input_names[i])] = input_sizes[i];
        return m;
    }

    // 🔴 obs 계약 대조 — 기동 시 «한 번» 부른다 (State_*.cpp 의 alg 생성 직후).
    //
    // 왜 있나: act() 는 입력 «이름» 이 없으면 throw 하지만 **크기는 안 봤다**.
    //   Ort::Value::CreateTensor<float>(mem, input_data.data(), input_sizes[i], ...)
    // 는 개수를 ONNX 에서(input_sizes) 가져오고 버퍼는 obs 에서(deploy.yaml) 가져온다.
    // 둘이 다르면 작을 땐 **범위 밖을 읽고**(UB) 클 땐 조용히 잘린다 — 어느 쪽도 에러가
    // 안 나고 「정책이 이상하다」로만 보인다. 그 상태로 50 Hz 를 돌리느니 여기서 죽는다.
    //
    // 로봇 무관한 순수 크기 검사라 공용(base)에 둔다 — g1 에서만 하면 나머지 로봇은
    // 계속 UB 로 남는다.
    // ONNX 에 구워진 obs 계약. 없으면 빈 문자열 (구버전 ONNX -> 경고 후 크기 검사만).
    std::string obs_contract() const
    {
        auto md = session->GetModelMetadata();
        Ort::AllocatorWithDefaultOptions alloc;
        auto v = md.LookupCustomMetadataMapAllocated("obs_contract", alloc);
        return v ? std::string(v.get()) : std::string();
    }

    std::string obs_contract_version() const
    {
        auto md = session->GetModelMetadata();
        Ort::AllocatorWithDefaultOptions alloc;
        auto v = md.LookupCustomMetadataMapAllocated("obs_contract_version", alloc);
        return v ? std::string(v.get()) : std::string();
    }

    void verify_inputs(const std::unordered_map<std::string, std::vector<float>>& obs,
                       const std::vector<ObsTermSpec>& terms = {}) const override
    {
        std::ostringstream bad;
        int n_bad = 0;
        std::ostringstream table;
        for (size_t i = 0; i < input_names.size(); ++i) {
            const std::string name(input_names[i]);
            const int64_t want = input_sizes[i];
            auto it = obs.find(name);
            const bool missing = (it == obs.end());
            const int64_t got = missing ? -1 : static_cast<int64_t>(it->second.size());
            table << "    " << (missing || got != want ? "x " : "o ")
                  << name << ": ONNX " << want
                  << " vs obs " << (missing ? std::string("(없음)") : std::to_string(got)) << "\n";
            if (missing) {
                bad << "\n  - 입력 '" << name << "' 이 obs 에 없다 (deploy.yaml 의 observations 그룹 이름 확인)";
                ++n_bad;
            } else if (got != want) {
                bad << "\n  - 입력 '" << name << "': ONNX 는 " << want
                    << " 를 요구하는데 obs 는 " << got << " 다 (차이 " << (got - want) << ")";
                ++n_bad;
            }
        }
        std::cout << "[obs contract] ONNX 입력 " << input_names.size() << " 개 대조\n"
                  << table.str() << std::flush;
        if (n_bad == 0) {
            std::cout << "[obs contract] 크기 일치" << std::endl;
            verify_terms(terms);        // 항 목록·순서·차원·history 까지 (계약이 있으면)
            return;
        }
        std::ostringstream msg;
        msg << "obs 계약 불일치 " << n_bad << " 건 — 기동을 거부한다." << bad.str()
            << "\n  deploy.yaml 의 observations 항별 (scale 길이 x history_length) 합이"
               " ONNX 입력과 같아야 한다."
            << "\n  이대로 돌리면 크기가 작을 땐 «범위 밖 읽기», 클 땐 «조용한 절단» 이라"
               " 에러 없이 정책이 쓰레기를 먹는다.";
        throw std::runtime_error(msg.str());
    }

    // 🔴 항 목록·순서·차원·history 대조. 크기(합)만 맞고 «순서가 뒤바뀐» 경우를 잡는다.
    //    joint_pos_rel / joint_vel_rel / last_action 이 전부 29 라 합만 보면 서로 교환돼도
    //    통과한다 — 그래서 이름까지 본다.
    //
    // 계약이 없는 ONNX(구버전)는 경고만 하고 통과시킨다. 기존 배포 슬롯을 한꺼번에
    // 못 쓰게 만들지 않기 위한 점진 도입이다.
    void verify_terms(const std::vector<ObsTermSpec>& terms) const
    {
        const std::string spec = obs_contract();
        if (spec.empty()) {
            std::cout << "[obs contract] \u26a0 이 ONNX 에는 계약이 없다 (구버전). 크기 검사만 수행했다.\n"
                      << "               학습 쪽 export 를 다시 하면 항 목록·순서까지 대조된다."
                      << std::endl;
            return;
        }
        // 호출부가 항 정보를 «안 넘긴» 경우 = 그 로봇의 State 가 아직 계약을 안 쓴다.
        // 여기서 죽이면 g1 외 5대(공용 base 를 쓰지만 호출부가 terms 를 안 준다)가 자기
        // ONNX 에 계약이 생기는 순간 통째로 기동 불가가 된다. 크기 검사는 이미 끝났으므로
        // 경고만 하고 통과시킨다 — «점진 도입» 은 양방향이어야 한다.
        if (terms.empty()) {
            std::cout << "[obs contract] \u26a0 ONNX 에 계약이 있는데 호출부가 항 정보를 안 넘겼다."
                      << " 크기 검사만 수행했다.\n"
                      << "               이 로봇의 State 에서 verify_inputs(obs, obs_terms) 로"
                         " 바꾸면 항 목록·순서까지 대조된다." << std::endl;
            return;
        }
        const auto want = parse_obs_contract(spec);
        const std::string ver = obs_contract_version();
        std::cout << "[obs contract] ONNX 계약 v" << (ver.empty() ? "?" : ver)
                  << " — 항 " << want.size() << " 개\n";

        std::ostringstream bad;
        int n_bad = 0;
        const size_t n = std::max(want.size(), terms.size());
        for (size_t i = 0; i < n; ++i) {
            const bool has_w = i < want.size(), has_g = i < terms.size();
            const std::string wn = has_w ? want[i].name : "(없음)";
            const std::string gn = has_g ? (terms[i].train_name.empty()
                                            ? "(train_term 미선언)" : terms[i].train_name) : "(없음)";
            const std::string dn = has_g ? terms[i].deploy_name : "-";
            bool okrow = has_w && has_g && wn == gn
                         && want[i].dim == terms[i].dim
                         && want[i].history == terms[i].history;
            char row[256];
            std::snprintf(row, sizeof row, "   %2zu %s %-22s %-22s %3d x%-3d\n",
                          i + 1, okrow ? "o" : "x", wn.c_str(), dn.c_str(),
                          has_w ? want[i].dim : (has_g ? terms[i].dim : 0),
                          has_w ? want[i].history : (has_g ? terms[i].history : 0));
            std::cout << row;
            if (okrow) continue;
            ++n_bad;
            if (!has_w)      bad << "\n  - " << (i + 1) << "번: 배포에만 있는 항 '" << dn << "'";
            else if (!has_g) bad << "\n  - " << (i + 1) << "번: 학습에만 있는 항 '" << wn << "'";
            else if (wn != gn)
                bad << "\n  - " << (i + 1) << "번: 학습은 '" << wn << "' 인데 배포는 '" << dn
                    << "' (train_term=" << gn << ") — 순서가 다르거나 train_term 이 틀렸다";
            else
                bad << "\n  - " << (i + 1) << "번 '" << wn << "': 학습 " << want[i].dim << "x"
                    << want[i].history << " vs 배포 " << terms[i].dim << "x" << terms[i].history;
        }
        if (n_bad == 0) {
            std::cout << "[obs contract] 통과 — 항 목록·순서·차원·history 가 학습과 일치" << std::endl;
            return;
        }
        std::ostringstream msg;
        msg << "obs 계약 불일치 " << n_bad << " 건 — 기동을 거부한다." << bad.str()
            << "\n  ONNX 에 구워진 계약(학습이 만든 것)과 deploy.yaml 의 observations 가 다르다."
            << "\n  합이 같아도 «순서» 가 다르면 정책이 뒤섞인 obs 를 먹는다 — 에러 없이 낙상으로만 보인다.";
        throw std::runtime_error(msg.str());
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // 이름 + 크기. 크기 검사가 없으면 아래 CreateTensor 가 obs 버퍼를 ONNX 개수만큼
        // 읽어 범위 밖을 건드린다(UB). 정상 경로에선 기동 시 verify_inputs 가 이미
        // 걸렀으므로 여기는 최후 보루다 — 정수 비교 하나라 50 Hz 에 무해하다.
        for (size_t i = 0; i < input_names.size(); ++i) {
            const std::string name(input_names[i]);
            auto it = obs.find(name);
            if (it == obs.end()) {
                throw std::runtime_error("Input name " + name + " not found in observations.");
            }
            if (static_cast<int64_t>(it->second.size()) != input_sizes[i]) {
                throw std::runtime_error(
                    "obs 크기 불일치: 입력 '" + name + "' ONNX " + std::to_string(input_sizes[i])
                    + " vs obs " + std::to_string(it->second.size())
                    + " (기동 시 verify_inputs 로 걸러졌어야 한다)");
            }
        }

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for(int i(0); i<input_names.size(); ++i)
        {
            const std::string name_str(input_names[i]);
            auto& input_data = obs.at(name_str);
            auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data.data(), input_sizes[i], input_shapes[i].data(), input_shapes[i].size());
            input_tensors.push_back(std::move(input_tensor));
        }

        // Run the model
        auto output_tensor = session->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(), input_tensors.size(), output_names.data(), 1);

        // Copy output data
        auto floatarr = output_tensor.front().GetTensorMutableData<float>();
        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action.data(), floatarr, output_shape[1] * sizeof(float));
        return action;
    }

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<int64_t> output_shape;
};
};
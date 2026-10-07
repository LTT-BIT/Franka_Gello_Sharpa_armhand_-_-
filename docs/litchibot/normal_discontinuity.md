# LitchiBot normal：独立 forwarding 控制

2026-10-07 最新设计已经替代此前的 minimum-interference severity/pattern gate。

normal/full fake 使用 **NormalHandAuthority + NormalHandRuntime + BasicNormalTarget**，不继承 validation latch，不使用 acquisition/slew/discontinuity/all-valid/pattern blockers。Start/Stop 只控制对应 forwarding。heartbeat 只诊断；坏 packet 只丢当前帧；source 或真实 control 断连进入 OFFLINE，恢复 READY。用户状态仅 READY/ACTIVE/OFFLINE。仅真实不可恢复设备/ownership/绑定错误保留内部故障阻止运行；原 SDK/driver/native protections 不改。

完整九项调用链、command 修改/drop/pause、Hard/Warning 边界、GUI/heartbeat 状态机与回归审计见 [normal_forwarding_audit.md](normal_forwarding_audit.md)。

之前的 severity_refactor、minimum_interference、slew-removal、heartbeat 修复等历史结果保留在 normal_discontinuity_test_results.json；它们不是当前 normal 运行策略。独立 diagnostic 与 supervised 的原文档只适用于各自模式。Manus/V4、SDK、vendor mapping、arm、tactile、recorder、camera 原链路不修改。

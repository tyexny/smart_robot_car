# 125 MHz PLL 输出的时序模型

当前正式工程仍以实际选定构建的 SDC 为准。第 16 轮采用以下边沿预算模型；小工程验证不代表整车时序通过。

以太网 PLL 电路、125 MHz 频率、50% 占空比和 90° 时钟均不修改。参考输入仍为板上 50 MHz。在再生 CLKOUT0 定义 8 ns 根时钟，CLKOUT1 从 CLKOUT0 生成 90° 时钟。setup/recovery 保留 0.500 ns 不确定度，所有其他显式时钟与 TX125 之间双向 hold/removal 也保留 0.500 ns 不确定度。同一物理时钟同一边沿的公共源变化不重复扣到 hold；没有增加 false path，既有 CDC、最大/最小延迟和 IO 约束保留。

依据为高云官方同系列 DDR 参考工程：参考输入和再生 PLL 输出分别建立 create_clock，再从输出建立下游生成时钟。这样避免将 50 MHz 参考时钟路由的早晚角差重复计入再生 125 MHz 的脉宽。DS981-1.1.3E 的 GW5AT-138 表 3-42 给出 PCLK≥100 MHz 时周期及周期间抖动小于 400 ps、输出占空比精度小于 50 mUI、静态相位偏移 ±50 ps。8 ns 的 50 mUI 为 0.4 ns；本试验保留 0.5 ns 输出变化预算。该预算以数据手册的低抖动晶振条件为前提；板级实测仍需验证。

独立 RGMII SDF 检查也把原有 0.2 ns 板级预算增至 **0.7 ns**，不得遗漏这部分输出变化。检查命令须带 `--budget-ns 0.7`。静态通过还要求内部 setup、hold、recovery、removal、RAM 最小脉宽及 DDR 结构检查均通过。

脉宽报告不自动扣除 clock uncertainty，因此审计额外从原生全局最小脉宽 slack 扣除 **0.500 ns**。为避免报告只列最差若干项而漏掉 TX 时钟，保守地对全局最小值统一扣除，不只处理已列出的 TX 行。原生正值小于 0.500 ns 仍会失败。该预算必须由构建清单携带，缺少预算时审计拒绝通过。

早期第14/15轮用 source latency 0～0.5 ns 代替边沿变化，未区分共同边沿。第16轮改用上面的 setup/cross-clock hold 模型。`probe_pll_uncertainty.py` 的原生小工程对照证实高云支持指定时钟对：同钟 hold 为 +0.247ns，加入跨钟预算后仍为 +0.247ns；ref50→TX125 hold 从 +0.880ns 收紧到 +0.415ns，反向 hold 也收紧。跨钟违例继续保留并失败，不能用模型修正掩盖真实跨域问题。原生报告和结果保存在 `output/four_tasks/pll_uncertainty_probe`。

隔离实验 `deployment/four_tasks/probe_pll_clock_root.py` 在原模型、PLL 输出根模型、带 0.5 ns 预算模型中均保留 ref50→TX125 和 TX125→ref50 各 16 条实际数据检查行，没有切断跨域数据检查。结果在 `output/four_tasks/pll_clock_root_probe/results.json`。本实验的位流不可用于小车。

官方资料：[器件数据手册](https://cdn.gowinsemi.com.cn/DS981E.pdf)、[时序约束指南](https://cdn.gowinsemi.com.cn/SUG940E.pdf)。整车最终结论必须读取对应冻结输入生成的完整报告，不能只按实验结果判定。

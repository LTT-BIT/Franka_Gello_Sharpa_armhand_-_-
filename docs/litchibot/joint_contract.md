# Sharpa command 关节契约

索引从 0 开始；所有 command 单位均为 rad。符号指映射输出对各具名输入通道的变化方向，不能等同物理坐标轴正方向。

| 侧 | index | joint | 输入及净方向 | SDK min | SDK max | 映射范围 |
| --- | --- | --- | --- | --- | --- | --- |
| left | 0 | thumb_CMC_FE | dexterous_cmc_yaw: +1 | -0.174500 | 1.919900 | 0.000000–1.919900 |
| left | 1 | thumb_CMC_AA | dexterous_cmc_swing: +1 | -0.349100 | 0.349100 | 0.000000–0.250000 |
| left | 2 | thumb_MCP_FE | thumb_mcp_flex: +1 | -0.523600 | 1.396300 | 0.000000–0.900000 |
| left | 3 | thumb_MCP_AA | 固定 0.0 | -0.349100 | 0.349100 | 0.000000–0.000000 |
| left | 4 | thumb_IP | thumb_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.100000 |
| left | 5 | index_MCP_FE | index_mcp_flex: +1 | -0.174533 | 1.570800 | 0.000000–1.570800 |
| left | 6 | index_MCP_AA | index_mcp_swing: -1 | -0.349100 | 0.349100 | -0.220000–0.220000 |
| left | 7 | index_PIP | index_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.745300 |
| left | 8 | index_DIP | index_dip_flex: +1 | 0.000000 | 1.396300 | 0.000000–1.396300 |
| left | 9 | middle_MCP_FE | middle_mcp_flex: +1 | -0.174533 | 1.570800 | 0.000000–1.570800 |
| left | 10 | middle_MCP_AA | middle_mcp_swing: -1 | -0.349100 | 0.349100 | -0.180000–0.180000 |
| left | 11 | middle_PIP | middle_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.745300 |
| left | 12 | middle_DIP | middle_dip_flex: +1 | 0.000000 | 1.396300 | 0.000000–1.396300 |
| left | 13 | ring_MCP_FE | ring_mcp_flex: +1 | -0.174533 | 1.570800 | 0.000000–1.570800 |
| left | 14 | ring_MCP_AA | ring_mcp_swing: -1 | -0.349100 | 0.349100 | -0.180000–0.180000 |
| left | 15 | ring_PIP | ring_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.745300 |
| left | 16 | ring_DIP | ring_dip_flex: +1 | 0.000000 | 1.396300 | 0.000000–1.396300 |
| left | 17 | pinky_CMC | pinky_mcp_swing: -1 | 0.000000 | 0.261800 | 0.000000–0.261800 |
| left | 18 | pinky_MCP_FE | pinky_mcp_flex: +1 | -0.174533 | 1.570800 | 0.000000–1.570800 |
| left | 19 | pinky_MCP_AA | pinky_mcp_swing: -1 | -0.349100 | 0.349100 | -0.220000–0.220000 |
| left | 20 | pinky_PIP | pinky_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.745300 |
| left | 21 | pinky_DIP | pinky_dip_flex: +1 | 0.000000 | 1.396300 | 0.000000–1.396300 |
| right | 0 | thumb_CMC_FE | dexterous_cmc_yaw: +1 | -0.174500 | 1.919900 | 0.000000–1.919900 |
| right | 1 | thumb_CMC_AA | dexterous_cmc_swing: +1 | -0.349100 | 0.349100 | 0.000000–0.250000 |
| right | 2 | thumb_MCP_FE | thumb_mcp_flex: +1 | -0.523600 | 1.396300 | 0.000000–0.900000 |
| right | 3 | thumb_MCP_AA | 固定 0.0 | -0.349100 | 0.349100 | 0.000000–0.000000 |
| right | 4 | thumb_IP | thumb_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.100000 |
| right | 5 | index_MCP_FE | index_mcp_flex: +1 | -0.174533 | 1.570800 | 0.000000–1.570800 |
| right | 6 | index_MCP_AA | index_mcp_swing: +1 | -0.349100 | 0.349100 | -0.220000–0.220000 |
| right | 7 | index_PIP | index_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.745300 |
| right | 8 | index_DIP | index_dip_flex: +1 | 0.000000 | 1.396300 | 0.000000–1.396300 |
| right | 9 | middle_MCP_FE | middle_mcp_flex: +1 | -0.174533 | 1.570800 | 0.000000–1.570800 |
| right | 10 | middle_MCP_AA | middle_mcp_swing: +1 | -0.349100 | 0.349100 | -0.180000–0.180000 |
| right | 11 | middle_PIP | middle_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.745300 |
| right | 12 | middle_DIP | middle_dip_flex: +1 | 0.000000 | 1.396300 | 0.000000–1.396300 |
| right | 13 | ring_MCP_FE | ring_mcp_flex: +1 | -0.174533 | 1.570800 | 0.000000–1.570800 |
| right | 14 | ring_MCP_AA | ring_mcp_swing: +1 | -0.349100 | 0.349100 | -0.180000–0.180000 |
| right | 15 | ring_PIP | ring_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.745300 |
| right | 16 | ring_DIP | ring_dip_flex: +1 | 0.000000 | 1.396300 | 0.000000–1.396300 |
| right | 17 | pinky_CMC | pinky_mcp_swing: -1 | 0.000000 | 0.261800 | 0.000000–0.261800 |
| right | 18 | pinky_MCP_FE | pinky_mcp_flex: +1 | -0.174533 | 1.570800 | 0.000000–1.570800 |
| right | 19 | pinky_MCP_AA | pinky_mcp_swing: +1 | -0.349100 | 0.349100 | -0.220000–0.220000 |
| right | 20 | pinky_PIP | pinky_pip_flex: +1 | 0.000000 | 1.745300 | 0.000000–1.745300 |
| right | 21 | pinky_DIP | pinky_dip_flex: +1 | 0.000000 | 1.396300 | 0.000000–1.396300 |

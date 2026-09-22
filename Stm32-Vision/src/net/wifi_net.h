#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// 网络模块：STA 连接（读取 config）+ 断线自动重连。
namespace net {

// 出网 MTU 上限（见 clamp_wan_mtu 的注释）。凡"按 MTU 留余量"的分片/分段都应从它派生，
// 别各自写死数值 —— 两处一旦不同步，修好一边就会顶坏另一边。
constexpr int kWanMtu = 1380;
// 把 STA 网卡 MTU 压到 kWanMtu（幂等，可反复调）。
// 本车所在网络的**公网路径** MTU 实测 1400（上行有隧道），而 lwIP 不做 PMTUD：按默认 1500
// 发出的段在这条路径上被**静默丢弃**，症状是"小请求全通、大 body 的 HTTPS POST 一直卡到超时"
// （ai_http 里的 code=-3/-5，握手/探针都正常）。lwIP 的发送段长 = min(pcb->mss, netif->mtu-40)，
// 所以压低网卡 MTU 即压低段长；SYN 里通告的 MSS 同步变小，反方向（服务端发给我们）也不再超限。
void clamp_wan_mtu();

void init();    // 开机调用一次：若已配置则发起连接（非阻塞）
void update();  // loop 中调用：断线后周期重连
void reconnect();  // 在线换网：按最新 cfg 断开重建 STA（不重启，BLE 保活）

bool is_connected();
IPAddress local_ip();

}  // namespace net

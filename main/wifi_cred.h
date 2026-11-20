#pragma once
/*
 * 留空 = 车自己开热点 "r3car"（默认，户外用这个）。
 * 填了 = 车连这个 WiFi（STA）。注意：
 *   - ESP32 只支持 2.4GHz，5G 的 SSID 连不上
 *   - 家里 HUAWEI-805 实测是 5GHz（ch149），所以这里保持空
 *   - 需要 Mac 保持联网时，可以填手机热点的 2.4G 名字
 *
 * 注意：这个文件会被编译进固件，密码等于明文存在板子里。
 * 用访客网络或者单独开一个 2.4G SSID 更稳妥 —— ESP32 只支持 2.4G，不是 5G。
 */
#define STA_SSID  ""
#define STA_PASS  ""

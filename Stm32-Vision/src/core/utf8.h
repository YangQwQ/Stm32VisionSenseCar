#pragma once
#include <stddef.h>
#include <string.h>

// UTF-8 尾巴边界回退(定长缓冲专用)。
// 往 `char buf[N]` 里 snprintf/strncpy 之后, 若末尾正好切在一个多字节字符的中间, 就把那段残缺
// 序列整个删掉。**不做这一步的后果**: 留下的孤立引导字节(如 0xE5)本身看着"合法", 但随后任何一处
// UTF-8 出口消毒都会把它换成 '?' —— ai_prompt 组请求给云端、ai_result 推手机、app_httpd 发 WS、
// blog 落日志, 四处都是。于是模型/手机看到的文本会莫名多一个问号, 且看不出是谁干的。
//
// 只处理"尾巴被切一半"这一件事: 整串是否合法 UTF-8 不归它管(那是上述出口消毒的职责)。
// 判据: 从尾往回数续字节(0x80-0xBF), 数够了说明尾巴是个完整字符 → 原样不动;
//       数不够说明引导字节后面的字节被截掉了 → 从引导字节处砍掉。
inline void utf8_clamp_tail(char* buf) {
  size_t n = strlen(buf), e = n, ncont = 0;
  while (e > 0) {
    unsigned char ch = (unsigned char)buf[e - 1];
    if ((ch & 0xC0) == 0x80) { e--; ncont++; continue; }        // 续字节: 继续回退
    int need;
    if (ch < 0x80) break;                                       // ASCII 结尾: 完整, e 仍 == n
    else if ((ch & 0xE0) == 0xC0) need = 1;
    else if ((ch & 0xF0) == 0xE0) need = 2;
    else if ((ch & 0xF8) == 0xF0) need = 3;
    else break;                                                 // 非法引导字节: 不动(e 仍 == n)
    // ⚠️ 完整时**必须把 e 退回 n**: 此刻 e 已退到该字符的引导字节处, 直接 `buf[e]=0` 会把最后一个
    //    字整个切掉、只剩一个孤立引导字节, 反而制造出上面说的那个 '?'(曾把整表任务名吃掉尾字)。
    if (ncont < need) e--;                                      // 续字节不足: 连同引导字节一起删
    else e = n;                                                 // 尾巴是个完整字符: 整串不动
    break;
  }
  if (e != n) buf[e] = 0;
}

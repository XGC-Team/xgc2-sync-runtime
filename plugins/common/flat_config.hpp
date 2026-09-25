// Reader for the flat `key = value` TOML lines the aggregator passes as a
// plugin's config (numbers, booleans, strings, and one-line number arrays).
// Every reader leaves `out` unchanged when the key is absent and returns
// false only when the key is present but malformed.
#pragma once

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace xgc_rt_config {

inline bool value(const std::string& text, const char* key, std::string* out) {
  size_t pos = 0;
  const size_t key_len = std::strlen(key);
  while ((pos = text.find(key, pos)) != std::string::npos) {
    const bool at_line_start = pos == 0 || text[pos - 1] == '\n';
    size_t p = pos + key_len;
    while (p < text.size() && text[p] == ' ') ++p;
    if (at_line_start && p < text.size() && text[p] == '=') {
      ++p;
      while (p < text.size() && text[p] == ' ') ++p;
      const size_t end = text.find('\n', p);
      *out = text.substr(p, end == std::string::npos ? std::string::npos : end - p);
      return true;
    }
    pos += key_len;
  }
  return false;
}

inline bool number(const std::string& text, const char* key, double* out) {
  std::string v;
  if (!value(text, key, &v)) return true;
  char* end = nullptr;
  const double d = std::strtod(v.c_str(), &end);
  if (end == v.c_str()) return false;
  *out = d;
  return true;
}

inline bool integer(const std::string& text, const char* key, int* out) {
  double d = *out;
  if (!number(text, key, &d) || d != static_cast<double>(static_cast<int>(d))) return false;
  *out = static_cast<int>(d);
  return true;
}

inline bool boolean(const std::string& text, const char* key, bool* out) {
  std::string v;
  if (!value(text, key, &v)) return true;
  if (v == "true") *out = true;
  else if (v == "false") *out = false;
  else return false;
  return true;
}

/* `key = [a, b, c]` with exactly `n` numbers. */
inline bool numbers(const std::string& text, const char* key, double* out, size_t n) {
  std::string v;
  if (!value(text, key, &v)) return true;
  if (v.size() < 2 || v.front() != '[' || v.back() != ']') return false;
  std::vector<double> got;
  const char* p = v.c_str() + 1;
  while (*p && *p != ']') {
    while (*p == ' ' || *p == ',') ++p;
    if (*p == ']') break;
    char* end = nullptr;
    const double d = std::strtod(p, &end);
    if (end == p) return false;
    got.push_back(d);
    p = end;
  }
  if (got.size() != n) return false;
  for (size_t i = 0; i < n; ++i) out[i] = got[i];
  return true;
}

/* A string value without its quotes; `fallback` when absent. */
inline std::string text_or(const std::string& text, const char* key, const char* fallback) {
  std::string v;
  if (!value(text, key, &v)) return fallback;
  if (v.size() >= 2 && v.front() == '"' && v.back() == '"') return v.substr(1, v.size() - 2);
  return v;
}

}  // namespace xgc_rt_config

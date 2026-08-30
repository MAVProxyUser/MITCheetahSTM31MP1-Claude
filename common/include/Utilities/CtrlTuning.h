/*!
 * @file CtrlTuning.h
 * @brief One source of truth for the controller's tuning values.
 *
 * OPEN-13. Every number that changes how this robot behaves used to live
 * ONLY in an environment variable - roughly two dozen of them, read with
 * `getenv("CTRL_X") ? atof(...) : <literal default>` scattered across eight
 * files. That is a configuration that exists nowhere you can read it: the
 * shipped yaml describes a robot nobody runs, the real robot is whatever
 * incantation the last harness happened to export, and an operator who
 * forgets one gets a different machine with no indication that anything
 * changed. This project has already lost results to exactly that class of
 * problem (a sweep hardcoding a flag in its own env block, and a recipe
 * whose panel label described a configuration it was not launching).
 *
 * So: the values live in a YAML file that ships with the binary, and the
 * environment variable becomes an OVERRIDE rather than the only home.
 *
 *   precedence:  environment variable  >  ctrl_tuning.yaml  >  code default
 *
 * The env override is kept deliberately - every sweep harness in this repo
 * drives configuration that way, and taking it away would break the tooling
 * that produced every measurement on record. What changes is that the
 * DEFAULT is now written down, in one file, next to the yaml the controller
 * already loads.
 *
 * Format is deliberately trivial (`key: value`, `#` comments) and parsed
 * here rather than through ParamHandler, because this must be readable from
 * headers included in the control loop's hot path without dragging a yaml
 * dependency into every translation unit. Values are cached on first read.
 */
#ifndef PROJECT_CTRLTUNING_H
#define PROJECT_CTRLTUNING_H

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>

namespace ctrl_tuning {

inline std::map<std::string, std::string>& table() {
  static std::map<std::string, std::string> t;
  static bool loaded = false;
  if (!loaded) {
    loaded = true;
    const char* p = getenv("CTRL_TUNING_YAML");
    const char* path = p ? p : "ctrl_tuning.yaml";
    FILE* f = fopen(path, "r");
    if (f) {
      char line[512];
      while (fgets(line, sizeof(line), f)) {
        char* h = strchr(line, '#');
        if (h) *h = '\0';
        char* c = strchr(line, ':');
        if (!c) continue;
        *c = '\0';
        std::string k(line), v(c + 1);
        auto trim = [](std::string& s) {
          size_t b = s.find_first_not_of(" \t\r\n");
          size_t e = s.find_last_not_of(" \t\r\n");
          s = (b == std::string::npos) ? std::string() : s.substr(b, e - b + 1);
        };
        trim(k); trim(v);
        if (!k.empty() && !v.empty()) t[k] = v;
      }
      fclose(f);
      printf("[ctrl_tuning] loaded %zu values from %s\n", t.size(), path);
      fflush(stdout);
    } else {
      printf("[ctrl_tuning] %s not found - code defaults, env still overrides\n",
             path);
      fflush(stdout);
    }
  }
  return t;
}

//! Raw lookup: environment first, then the yaml, else nullptr.
inline const char* raw(const char* key) {
  const char* e = getenv(key);
  if (e && *e) return e;
  auto& t = table();
  auto it = t.find(key);
  return (it == t.end()) ? nullptr : it->second.c_str();
}

inline double num(const char* key, double dflt) {
  const char* v = raw(key);
  return v ? atof(v) : dflt;
}

inline int integer(const char* key, int dflt) {
  const char* v = raw(key);
  return v ? atoi(v) : dflt;
}

inline bool flag(const char* key, bool dflt) {
  const char* v = raw(key);
  return v ? (atoi(v) != 0) : dflt;
}

}  // namespace ctrl_tuning

#endif  // PROJECT_CTRLTUNING_H

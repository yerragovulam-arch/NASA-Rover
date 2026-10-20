// Tiny dependency-free test framework shared by all C++ tests.
//
//   TEST(name) { CHECK(x); CHECK_NEAR(a, b, tol); REQUIRE(ptr); INFO("..%d", n); }
//   XFAIL_TEST(name, "why") { ... }   // known issue: expected to FAIL today.
//
// Results:  PASS | FAIL | XFAIL (known issue still present) | XPASS (known issue
// is fixed -> remove the XFAIL marker).  Exit code is non-zero on FAIL or XPASS.
// Flags: --list, --filter <substr>, --json <file>, -v
#pragma once
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace tf {

struct Case {
  std::string name;
  std::function<void()> fn;
  std::string xfail;  // empty = normal test
};
struct Result {
  std::string name, status, xfail, detail;
  std::vector<std::string> infos;
};
struct Abort {};

inline std::vector<Case> & registry() { static std::vector<Case> r; return r; }
inline std::vector<std::string> & failures() { static std::vector<std::string> f; return f; }
inline std::vector<std::string> & infos() { static std::vector<std::string> f; return f; }

struct Reg {
  Reg(const char * n, std::function<void()> f, const char * x = "") {
    registry().push_back({n, std::move(f), x});
  }
};

inline std::string fmt(const char * f, ...) {
  char buf[2048];
  va_list a; va_start(a, f); vsnprintf(buf, sizeof buf, f, a); va_end(a);
  return buf;
}
inline void fail(const char * file, int line, const std::string & m) {
  const char * base = std::strrchr(file, '/');
  failures().push_back(fmt("%s:%d: %s", base ? base + 1 : file, line, m.c_str()));
}
inline void info(const std::string & m) { infos().push_back(m); }

inline std::string jesc(const std::string & s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if (static_cast<unsigned char>(c) >= 0x20) o += c;
  }
  return o;
}

inline int run_main(int argc, char ** argv, const char * suite,
                    const std::function<void()> & per_test_reset = nullptr) {
  std::string filter, json;
  bool list = false, verbose = false;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--list")) list = true;
    else if (!std::strcmp(argv[i], "-v")) verbose = true;
    else if (!std::strcmp(argv[i], "--filter") && i + 1 < argc) filter = argv[++i];
    else if (!std::strcmp(argv[i], "--json") && i + 1 < argc) json = argv[++i];
  }
  if (list) { for (auto & c : registry()) std::printf("%s\n", c.name.c_str()); return 0; }
  std::vector<Result> results;
  int nfail = 0, nxpass = 0, nxfail = 0, npass = 0;
  for (auto & c : registry()) {
    if (!filter.empty() && c.name.find(filter) == std::string::npos) continue;
    failures().clear(); infos().clear();
    if (per_test_reset) per_test_reset();
    try { c.fn(); }
    catch (const Abort &) {}
    catch (const std::exception & e) { failures().push_back(std::string("exception: ") + e.what()); }
    if (per_test_reset) per_test_reset();
    const bool bad = !failures().empty();
    Result r{c.name, "", c.xfail, "", infos()};
    for (auto & f : failures()) r.detail += f + "\n";
    if (c.xfail.empty()) { r.status = bad ? "FAIL" : "PASS"; (bad ? nfail : npass)++; }
    else { r.status = bad ? "XFAIL" : "XPASS"; (bad ? nxfail : nxpass)++; }
    std::printf("[%-5s] %s::%s\n", r.status.c_str(), suite, c.name.c_str());
    if (r.status == "XFAIL") std::printf("         known issue: %s\n", c.xfail.c_str());
    if (r.status == "XPASS") std::printf("         known issue appears FIXED -> convert to TEST: %s\n", c.xfail.c_str());
    if (r.status == "FAIL" || verbose) for (auto & f : failures()) std::printf("         %s\n", f.c_str());
    for (auto & s : r.infos) std::printf("         info: %s\n", s.c_str());
    results.push_back(r);
  }
  std::printf("-- %s: %d pass, %d fail, %d xfail (known issues), %d xpass\n",
              suite, npass, nfail, nxfail, nxpass);
  if (!json.empty()) {
    FILE * f = std::fopen(json.c_str(), "w");
    if (f) {
      std::fprintf(f, "{\"suite\":\"%s\",\"results\":[", suite);
      for (size_t i = 0; i < results.size(); ++i) {
        auto & r = results[i];
        std::fprintf(f, "%s{\"name\":\"%s\",\"status\":\"%s\",\"xfail\":\"%s\",\"detail\":\"%s\",\"infos\":[",
                     i ? "," : "", jesc(r.name).c_str(), r.status.c_str(), jesc(r.xfail).c_str(), jesc(r.detail).c_str());
        for (size_t k = 0; k < r.infos.size(); ++k)
          std::fprintf(f, "%s\"%s\"", k ? "," : "", jesc(r.infos[k]).c_str());
        std::fprintf(f, "]}");
      }
      std::fprintf(f, "]}\n");
      std::fclose(f);
    }
  }
  return (nfail || nxpass) ? 1 : 0;
}

}  // namespace tf

#define TF_CAT2(a, b) a##b
#define TF_CAT(a, b) TF_CAT2(a, b)
#define TEST(name) \
  static void name(); static ::tf::Reg TF_CAT(reg_, name)(#name, name); static void name()
#define XFAIL_TEST(name, reason) \
  static void name(); static ::tf::Reg TF_CAT(reg_, name)(#name, name, reason); static void name()

#define CHECK(c) do { if (!(c)) ::tf::fail(__FILE__, __LINE__, "CHECK failed: " #c); } while (0)
#define CHECK_MSG(c, ...) do { if (!(c)) ::tf::fail(__FILE__, __LINE__, \
  std::string("CHECK failed: " #c " -- ") + ::tf::fmt(__VA_ARGS__)); } while (0)
#define REQUIRE(c) do { if (!(c)) { ::tf::fail(__FILE__, __LINE__, "REQUIRE failed: " #c); throw ::tf::Abort(); } } while (0)
#define CHECK_NEAR(a, b, tol) do { double _a = (a), _b = (b), _t = (tol); \
  if (!(std::fabs(_a - _b) <= _t)) ::tf::fail(__FILE__, __LINE__, ::tf::fmt( \
  "CHECK_NEAR failed: " #a " = %.6g vs " #b " = %.6g (tol %.3g)", _a, _b, _t)); } while (0)
#define INFO(...) ::tf::info(::tf::fmt(__VA_ARGS__))

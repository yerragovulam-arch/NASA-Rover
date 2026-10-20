"""Minimal test harness for the simulation tests (same result schema as the C++ framework).

    @test            normal test
    @xfail("why")    known issue: expected to fail today (XPASS = fixed, remove marker)
    @slow            only runs with --long
Inside a test: chk(cond, msg), require(cond, msg), info(msg), near(a, b, tol, msg)
"""
import argparse
import json
import sys
import time
import traceback

REGISTRY = []
_cur = {"fails": [], "infos": []}


class Abort(Exception):
    pass


def test(fn=None, *, xfail=None, slow=False):
    def deco(f):
        REGISTRY.append(dict(name=f.__name__, fn=f, xfail=xfail or "", slow=slow, module=f.__module__))
        return f
    return deco(fn) if fn else deco


def xfail(reason, slow=False):
    return test(xfail=reason, slow=slow)


def slow(fn):
    return test(fn, slow=True)


def chk(cond, msg="check failed"):
    if not cond:
        _cur["fails"].append(msg)
    return bool(cond)


def require(cond, msg="require failed"):
    if not cond:
        _cur["fails"].append(msg)
        raise Abort()


def near(a, b, tol, msg=""):
    return chk(abs(a - b) <= tol, f"{msg} {a:.5g} vs {b:.5g} (tol {tol:.3g})")


def info(msg):
    _cur["infos"].append(msg)


def run(suite, argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--filter", default="")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--json", default="")
    ap.add_argument("--long", action="store_true", help="include slow tests (endurance, repeatability x20)")
    ap.add_argument("-v", action="store_true")
    a = ap.parse_args(argv)
    cases = [c for c in REGISTRY if a.filter in c["name"]]
    if a.list:
        for c in cases:
            print(c["name"])
        return 0
    counts = dict(PASS=0, FAIL=0, XFAIL=0, XPASS=0, SKIP=0)
    results = []
    for c in cases:
        if c["slow"] and not a.long:
            counts["SKIP"] += 1
            print(f"[SKIP ] {suite}::{c['name']}  (slow; use --long)")
            results.append(dict(name=c["name"], status="SKIP", xfail=c["xfail"], detail="", infos=[], secs=0))
            continue
        _cur["fails"], _cur["infos"] = [], []
        t0 = time.time()
        try:
            c["fn"]()
        except Abort:
            pass
        except Exception:
            _cur["fails"].append("exception: " + traceback.format_exc().strip().splitlines()[-1])
            if a.v:
                traceback.print_exc()
        bad = bool(_cur["fails"])
        if c["xfail"]:
            status = "XFAIL" if bad else "XPASS"
        else:
            status = "FAIL" if bad else "PASS"
        counts[status] += 1
        secs = time.time() - t0
        print(f"[{status:<5}] {suite}::{c['name']}  ({secs:.1f}s)")
        if status == "XFAIL":
            print(f"         known issue: {c['xfail']}")
        if status == "XPASS":
            print(f"         known issue appears FIXED -> convert to @test: {c['xfail']}")
        if status == "FAIL" or a.v:
            for f in _cur["fails"]:
                print(f"         {f}")
        for i in _cur["infos"]:
            print(f"         info: {i}")
        results.append(dict(name=c["name"], status=status, xfail=c["xfail"], detail="\n".join(_cur["fails"]),
                            infos=list(_cur["infos"]), secs=round(secs, 2)))
        sys.stdout.flush()
    print(f"-- {suite}: {counts['PASS']} pass, {counts['FAIL']} fail, {counts['XFAIL']} xfail (known issues), "
          f"{counts['XPASS']} xpass, {counts['SKIP']} skipped")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(dict(suite=suite, results=results), f)
    return 1 if (counts["FAIL"] or counts["XPASS"]) else 0

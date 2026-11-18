#!/usr/bin/env python3
"""Collects the gate lines of the release-candidate jobs into crates/bowshock-cfd/fixtures/release_candidate_gates.txt and prints the table (DESIGN3D item 42).
usage: collect_gates.py SHA group=logfile ..."""
import sys, re, collections
sha = sys.argv[1]
out = [f"# Release-candidate gate lines, one snapshot for the whole file: cfd-3d {sha}. Groups: tools/release_gates.sh."]
count = collections.OrderedDict()
bad = []
for arg in sys.argv[2:]:
    group, path = arg.split("=", 1)
    out.append(f"\n## {group}")
    for line in open(path, errors="replace"):
        line = line.rstrip("\n")
        if not re.match(r"^(GATE|INFO|test result|thread .* panicked)", line):
            continue
        out.append(line)
        m = re.match(r"^(GATE|INFO) (\S+) (\S+) (.*)$", line)
        if not m:
            continue
        kind, gid = m.group(1), m.group(2)
        c = count.setdefault(gid, collections.Counter())
        if kind == "INFO":
            c["INFO"] += 1
        elif " FAIL KNOWN " in line:
            c["KNOWN"] += 1; bad.append(("KNOWN", line))
        elif line.endswith(" PASS") or re.search(r" PASS( |$)", line):
            c["PASS"] += 1
        elif "FAIL" in line:
            c["FAIL"] += 1; bad.append(("FAIL", line))
        else:
            c["INFO"] += 1
open("crates/bowshock-cfd/fixtures/release_candidate_gates.txt", "w").write("\n".join(out) + "\n")
print(f"| gate | PASS | FAIL | KNOWN | INFO |\n|---|---|---|---|---|")
for gid, c in count.items():
    print(f"| {gid} | {c['PASS']} | {c['FAIL']} | {c['KNOWN']} | {c['INFO']} |")
print()
for k, l in bad:
    print(k, l)

#!/usr/bin/env python3
# Print a VHDL file with comments removed and all whitespace collapsed, so
# two versions can be compared for code changes only (used to show that a
# comment-only edit changed no code):
#   git show HEAD~1:rtl/x.vhd > old.vhd
#   diff <(python3 -I sim/check_comment_only.py old.vhd) \
#        <(python3 -I sim/check_comment_only.py rtl/x.vhd) && echo "code unchanged"
import re, sys
src = open(sys.argv[1]).read()
out = []
for line in src.splitlines():
    # remove "--" comments that are not inside a string or character literal
    res, i, in_str = [], 0, False
    while i < len(line):
        c = line[i]
        if c == '"':
            in_str = not in_str
        elif not in_str and c == "'" and i + 2 < len(line) and line[i + 2] == "'":
            res.append(line[i:i + 3]); i += 3; continue      # character literal like '-'
        elif not in_str and line.startswith('--', i):
            break
        res.append(c); i += 1
    out.append(''.join(res))
print(' '.join(' '.join(out).split()))

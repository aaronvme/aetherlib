#!/usr/bin/env python3
"""Fail if API.md documents a default that the header does not actually have.

Documentation drift on a tuned constant is worse than no documentation: it is
read as authoritative. This found `thetaResampleVariance` documented as 0.01
against a header default of 0.001, and a `resampleThreshold` description that
claimed a gate the code does not apply.

It also fails when config.cpp ships a different value for a documented field
and API.md does not say so. That is the trap that made the MCL search ramp dead
code: the header default read as though a fix had landed while the robot ran the
old value, so `distResampleVariance` was documented as 0.15 and shipped at 1.5.
Overriding a default is normal; leaving a reader to assume the default is live
is not.

Only mismatches fail. A documented field that cannot be located is reported and
tolerated -- aggregate initialisers (`{10, 0, 0.3}`) and macros legitimately do
not parse as scalar defaults.

Both markdown tables and fenced struct listings are read. Checking only tables
left every struct that API.md documents as a code block unchecked, which is most
of the tuning surface: `CharacterizeConfig` sat at a documented 0.6 V/s against
a header 2.0, and a dead `stepVolts` stayed in the docs after the code dropped
it.
"""
import glob
import io
import re
import sys

SCALAR = re.compile(
    r'^\s*(?:float|int|bool|std::uint32_t|uint32_t|std::int32_t)\s+(\w+)\s*'
    r'(?:=\s*|\{)\s*([-\w.]+?)f?\s*[;}]', re.M)
ROW = re.compile(r'^\|\s*`([A-Za-z_][\w:]*)`\s*\|\s*`([^`|]+)`\s*\|', re.M)
DESIGNATED = re.compile(r'^\s*\.(\w+)\s*=\s*([-\w.]+?)f?\s*,', re.M)
FENCE = re.compile(r'^```cpp$(.*?)^```$', re.M | re.S)
STRUCT = re.compile(r'^struct\s+(\w+)\s*\{(.*?)^\};', re.M | re.S)
CONFIG = 'src/gamespecific/config.cpp'


def norm(value):
    value = value.strip().strip('`')
    if value in ('true', 'false'):
        return value
    try:
        return '%g' % float(value)
    except ValueError:
        return value


def main():
    defaults = {}
    for path in glob.glob('include/chad/**/*.hpp', recursive=True):
        text = io.open(path, encoding='utf-8', errors='replace').read()
        for m in SCALAR.finditer(text):
            defaults.setdefault(m.group(1), set()).add(norm(m.group(2)))

    shipped = {}
    for m in DESIGNATED.finditer(io.open(CONFIG, encoding='utf-8', errors='replace').read()):
        shipped.setdefault(m.group(1), set()).add(norm(m.group(2)))

    api = io.open('API.md', encoding='utf-8', errors='replace').read()
    rows_by_field = {}
    for line in api.splitlines():
        m = re.match(r'^\|\s*`([A-Za-z_][\w:]*)`\s*\|', line)
        if m:
            rows_by_field.setdefault(m.group(1), line)

    bad, missing, checked, silent = [], [], 0, []
    for field, documented in ROW.findall(api):
        key = field.split('::')[-1]
        if key not in defaults:
            missing.append((field, documented))
            continue
        checked += 1
        if norm(documented) not in defaults[key]:
            bad.append((field, norm(documented), sorted(defaults[key])))
            continue
        live = shipped.get(key)
        if live and not (live & defaults[key]):
            row = rows_by_field.get(field, '')
            if 'config.cpp' not in row:
                silent.append((field, sorted(defaults[key]), sorted(live)))

    header_structs = {}
    for path in glob.glob('include/chad/**/*.hpp', recursive=True):
        text = io.open(path, encoding='utf-8', errors='replace').read()
        for name, body in STRUCT.findall(text):
            fields = header_structs.setdefault(name, {})
            for f in SCALAR.finditer(body):
                fields[f.group(1)] = norm(f.group(2))

    struct_bad, struct_missing, struct_checked = [], [], 0
    for block in FENCE.findall(api):
        for name, body in STRUCT.findall(block):
            if name not in header_structs:
                struct_missing.append(name)
                continue
            for f in SCALAR.finditer(body):
                field, documented = f.group(1), norm(f.group(2))
                live = header_structs[name].get(field)
                if live is None:
                    struct_missing.append(f'{name}::{field}')
                    continue
                struct_checked += 1
                if documented != live:
                    struct_bad.append((f'{name}::{field}', documented, live))

    print(f'{checked} documented defaults checked against include/chad/**')
    print(f'{struct_checked} struct-block defaults checked field by field')
    if struct_missing:
        print(f'{len(struct_missing)} documented in a code block but not found '
              f'in a header struct (tolerated):')
        for name in struct_missing:
            print(f'    {name}')
    for field, documented, live in struct_bad:
        print(f'  MISMATCH {field}: API.md={documented} header={live}')
    if missing:
        print(f'{len(missing)} not locatable as a scalar default (tolerated):')
        for field, documented in missing:
            print(f'    {field} = {documented}')
    for field, documented, header in bad:
        print(f'  MISMATCH {field}: API.md={documented} header={header}')
    for field, header, live in silent:
        print(f'  UNDOCUMENTED OVERRIDE {field}: header={header} '
              f'config.cpp={live}, and API.md does not mention it')
    if bad or silent or struct_bad:
        print(f'FAILED ({len(bad) + len(struct_bad)} mismatched, '
              f'{len(silent)} silent overrides)')
        return 1
    print('defaults agree')
    return 0


if __name__ == '__main__':
    sys.exit(main())

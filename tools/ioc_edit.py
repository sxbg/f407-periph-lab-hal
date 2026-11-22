"""Canonical .ioc editor for f407-periph-lab-hal.

Implements the roundtrip-proven layout rules (knowledge #12): every key
in byte-sorted order, exactly one board=custom terminator at the end,
Mcu.IPn renumbered for the full IP set, and <IP>.IPParameters whitelist
entries added before (or with) any parameter key. CubeMX drops keys
whose name is absent from the whitelist CSV, and treats board=custom as
the file terminator -- both traps are handled here mechanically.

Usage:
  python tools/ioc_edit.py <ioc-path> <spec.json>
where spec.json:
  { "ips": ["SPI1", ...],                       # IP names to enable
    "params": {"SPI1.Mode": "SPI_MODE_MASTER"}, # full key -> value
    "whitelist": {"SPI1": ["Mode", ...]}        # CSV entries to add
  }

The file is rewritten with CRLF endings and validated afterwards by a
real CubeMX load/generate (asm cubegen regen) -- the generated Core/Src
sources are the only accepted proof.
"""
import json
import sys


def main() -> int:
    ioc_path, spec_path = sys.argv[1], sys.argv[2]
    with open(spec_path, "r", encoding="utf-8") as f:
        spec = json.load(f)

    with open(ioc_path, "rb") as f:
        raw = f.read()
    if b"\x00" in raw:
        print("FATAL: null byte in ioc", file=sys.stderr)
        return 2
    lines = [l.rstrip(b"\r\n") for l in raw.split(b"\r\n")]
    if lines and lines[-1] == b"":
        lines.pop()

    # Parse key=value (first '=' splits; values may contain escaped \:).
    kv = {}
    order_dup = []
    for i, line in enumerate(lines):
        if b"=" not in line:
            if line:
                order_dup.append((i, line))
            continue
        key, val = line.split(b"=", 1)
        if key in kv:
            order_dup.append((i, line))
        kv[key] = val
    if order_dup:
        print("WARN: non-key or duplicate lines dropped: %d" % len(order_dup),
              file=sys.stderr)

    # 1. IP set: add requested names, rebuild the numeric enumeration.
    ips = set()
    for k, v in kv.items():
        if k.startswith(b"Mcu.IP") and k != b"Mcu.IPNb":
            ips.add(v.decode("ascii"))
    for ip in spec.get("ips", []):
        ips.add(ip)
    # Canonical prefix order (NVIC, RCC, SYS first, then alphabetical) --
    # the order CubeMX itself writes; indices are only a stable mapping.
    prefixes = [x for x in ("NVIC", "RCC", "SYS") if x in ips]
    rest = sorted(ips - set(prefixes))
    seq = prefixes + rest
    for k in [k for k in kv if k.startswith(b"Mcu.IP")]:
        del kv[k]
    kv[b"Mcu.IPNb"] = str(len(seq)).encode("ascii")
    for idx, name in enumerate(seq):
        kv[b"Mcu.IP%d" % idx] = name.encode("ascii")

    # 2. Whitelists: add requested param names to <IP>.IPParameters CSV.
    for ip, entries in spec.get("whitelist", {}).items():
        key = ("%s.IPParameters" % ip).encode("ascii")
        if key in kv:
            have = [e for e in kv[key].decode("ascii").split(",") if e]
            merged = have + [e for e in entries if e not in have]
            kv[key] = ",".join(merged).encode("ascii")
        else:
            kv[key] = ",".join(entries).encode("ascii")

    # 3. Parameter keys (plain overwrite / insert).
    for k, v in spec.get("params", {}).items():
        kv[k.encode("ascii")] = v.encode("ascii")

    # 4. Re-emit: byte-sorted keys, one board=custom terminator last.
    # (board=custom parsed as a normal key above must NOT be emitted in
    # the sorted body -- CubeMX treats it as the file terminator.)
    kv.pop(b"board", None)
    out = sorted(kv.items(), key=lambda item: item[0])
    body = [k + b"=" + v for k, v in out]
    body.append(b"board=custom")
    with open(ioc_path, "wb") as f:
        f.write(b"\r\n".join(body) + b"\r\n")

    print("ips=%d keys=%d" % (len(seq), len(body)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Genera los vectores de test/test_frame desde la referencia (collar_sub.py) — nada escrito a mano.

  1. vectors.h (PÚBLICO): bits de frame_bits() con un ID DE EJEMPLO (TEST_ID, el mismo -DT60_ID del env
     native) para casos fijos (bordes 0/99/soltar/luz) + 24 aleatorios (semilla fija), más pruebas negativas
     (ID ajeno, checksum malo = XOR del último byte de la referencia).
  2. vectors_local.h (NO versionado, .gitignore): RAW_Data de los .sub generados con gen() y CONFIRMADOS en el
     collar (2026-10-20). Llevan el ID REAL del collar, por eso no se publican; el test que los usa se salta si
     el archivo no existe. Los parámetros (seq/modo/potencia/tramas) salen del decode de collar_sub.py.

Uso:  python3 -I tools/gen_vectors.py [ruta/collar_sub.py] [dir_gen]
"""
import importlib.util, os, random, sys

HERE = os.path.dirname(os.path.abspath(__file__))
REF = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/.claude/skills/t-embed-cc1101/collar_sub.py")
GEN_DIR = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser("~/.config/tembed/gen")
TEST_ID = (0x12, 0x34)  # = -DT60_ID=0x1234 de [env:native]; NO es el collar real
GOLDEN = ["C_vib10.sub", "C_tono.sub", "C_luz_on.sub", "C_luz_off.sub", "D_vib10_1t.sub"]

spec = importlib.util.spec_from_file_location("collar_sub", REF)
cs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cs)


def frames_in(path):
    """Mismo algoritmo que collar_sub.decode_file, devolviendo (seq, modo, potencia) por trama."""
    _, raw = cs.load(path)
    bits = "".join(map(str, [b for burst in cs.bursts(raw, cs.BIT_US, gap_units=10**9) for b in burst]))
    head = cs.SYNC[2:] + "".join(f"{b:08b}" for b in cs.ID)
    out, k = [], bits.find(head)
    while k >= 0:
        d = bits[k + len(head): k + len(head) + 32]
        seq, mode, pw, ck = (int(d[i:i + 8], 2) for i in range(0, 32, 8))
        assert (sum(cs.ID) + seq + mode + pw) & 0xFF == ck, path
        out.append((seq, mode, pw))
        k = bits.find(head, k + 1)
    return out


def main():
    rnd = random.Random(0x7460)
    ID = TEST_ID
    cases = [  # (seq, modo, potencia, id, chk_xor, etiqueta)
        (0xE3, 0x21, 10, ID, 0, "vibrar 10"),
        (0xE3, 0x31, 0, ID, 0, "tono 0"),
        (0x37, 0x41, 1, ID, 0, "luz on"),
        (0x3C, 0x41, 0, ID, 0, "luz off"),
        (0x00, 0x11, 0, ID, 0, "shock 0, seq 0"),
        (0xFF, 0x11, 99, ID, 0, "shock 99, seq FF"),
        (0xE4, 0x01, 0xFF, ID, 0, "soltar"),
        (0x80, 0x21, 60, ID, 0, "vibrar 60"),
        (0xE3, 0x21, 60, (ID[0], ID[1] ^ 0x01), 0, "NEG id ajeno"),
        (0xE3, 0x21, 60, ID, 0x5A, "NEG checksum ^ 0x5A"),
    ]
    for _ in range(24):
        cases.append((rnd.randrange(256), rnd.choice([0x11, 0x21, 0x31, 0x41, 0x12, 0x01]),
                      rnd.randrange(100), ID, 0, "aleatorio"))

    lines = ["// GENERADO por tools/gen_vectors.py desde collar_sub.py — no editar a mano.",
             f"// ID DE EJEMPLO 0x{ID[0]:02X}{ID[1]:02X} (= T60_ID del env native), no el del collar.",
             "#pragma once", "#include <stdint.h>", "",
             "struct FrameVec { uint8_t seq, mode, power, idHi, idLo, chkXor; const char *label; const char *bits; };",
             "static const FrameVec kFrameVecs[] = {"]
    for seq, mode, pw, ident, cx, label in cases:
        b = cs.frame_bits(seq, mode, pw, ident=ident)
        if cx:
            b = b[:-8] + f"{int(b[-8:], 2) ^ cx:08b}"
        assert len(b) == 1104
        lines.append(f'  {{0x{seq:02X}, 0x{mode:02X}, {pw}, 0x{ident[0]:02X}, 0x{ident[1]:02X}, 0x{cx:02X}, "{label}",')
        lines.append(f'   "{b}"}},')
    lines += ["};", ""]
    dst = os.path.join(HERE, "..", "test", "test_frame", "vectors.h")
    with open(dst, "w") as f:
        f.write("\n".join(lines))
    print(f"{len(cases)} tramas -> {os.path.normpath(dst)}")

    if not os.path.isdir(GEN_DIR):
        print(f"(sin {GEN_DIR}: no se generan los vectores locales de capturas reales)")
        return
    loc = ["// GENERADO por tools/gen_vectors.py — NO VERSIONAR: contiene el ID real del collar (.gitignore).",
           "#pragma once", "#include <stdint.h>", "",
           "struct GoldenPress { const char *file; uint8_t idHi, idLo, seq, mode, power; int frames; bool release;"
           " const int32_t *raw; int n; };"]
    entries = []
    for name in GOLDEN:
        path = os.path.join(GEN_DIR, name)
        fr = frames_in(path)
        release = fr[-1][1:] == (0x01, 0xFF)
        body = fr[:-1] if release else fr
        assert body and all(x == body[0] for x in body), (name, fr)
        _, raw = cs.load(path)
        sym = name.split(".")[0]
        loc.append(f"static const int32_t kRaw_{sym}[] = {{")
        for i in range(0, len(raw), 24):
            loc.append("  " + ", ".join(map(str, raw[i:i + 24])) + ",")
        loc.append("};")
        seq, mode, pw = body[0]
        entries.append(f'  {{"{name}", 0x{cs.ID[0]:02X}, 0x{cs.ID[1]:02X}, 0x{seq:02X}, 0x{mode:02X}, {pw}, {len(body)},'
                       f' {"true" if release else "false"}, kRaw_{sym}, {len(raw)}}},')
        print(f"{name}: seq=0x{seq:02X} modo=0x{mode:02X} pot={pw} tramas={len(body)} soltar={release} raw={len(raw)}")
    loc += ["static const GoldenPress kGolden[] = {", *entries, "};", ""]
    dst = os.path.join(HERE, "..", "test", "test_frame", "vectors_local.h")
    with open(dst, "w") as f:
        f.write("\n".join(loc))
    print(f"{len(entries)} golden (ID real) -> {os.path.normpath(dst)}")


if __name__ == "__main__":
    main()

"""Markdown sencillo -> .docx (python-docx) para los documentos de la tesis: títulos #..####, párrafos, viñetas «- », listas «1. »,
tablas con barras, imágenes ![pie](ruta) y énfasis **negrita**, *cursiva* y `código`. Las rutas de imagen son relativas al .md.
Uso: python3 tools/md_a_docx.py entrada.md [salida.docx]"""
import re, sys
from pathlib import Path

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.shared import Cm, Pt

PATRON = re.compile(r"(\*\*[^*]+\*\*|\*[^*]+\*|`[^`]+`)")


def en_linea(par, texto, tam=None):
    for trozo in PATRON.split(texto):
        if not trozo:
            continue
        if trozo.startswith("**"):
            r = par.add_run(trozo[2:-2]); r.bold = True
        elif trozo.startswith("`"):
            r = par.add_run(trozo[1:-1]); r.font.name = "Consolas"
        elif trozo.startswith("*") and len(trozo) > 2:
            r = par.add_run(trozo[1:-1]); r.italic = True
        else:
            r = par.add_run(trozo)
        if tam:
            r.font.size = Pt(tam)


def tabla(doc, filas):
    celdas = [[c.strip() for c in f.strip().strip("|").split("|")] for f in filas if not re.match(r"^\|?\s*:?-{2,}", f.strip())]
    n = max(len(f) for f in celdas)
    t = doc.add_table(rows=len(celdas), cols=n)
    t.style = "Table Grid"; t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, f in enumerate(celdas):
        for j in range(n):
            p = t.cell(i, j).paragraphs[0]
            en_linea(p, f[j] if j < len(f) else "", 8.5)
            if i == 0:
                for r in p.runs:
                    r.bold = True
    doc.add_paragraph()


def convertir(md: Path, salida: Path):
    doc = Document()
    st = doc.styles["Normal"]; st.font.name = "Times New Roman"; st.font.size = Pt(12)
    for s in doc.sections:
        s.left_margin = s.right_margin = Cm(2.5)
    lineas = md.read_text(encoding="utf-8").splitlines()
    i = 0
    while i < len(lineas):
        ln = lineas[i]
        if ln.startswith("|"):
            bloque = []
            while i < len(lineas) and lineas[i].startswith("|"):
                bloque.append(lineas[i]); i += 1
            tabla(doc, bloque); continue
        m = re.match(r"^(#{1,4})\s+(.*)", ln)
        img = re.match(r"^!\[([^\]]*)\]\(([^)]+)\)", ln.strip())
        if m:
            doc.add_heading(m.group(2).replace("**", ""), level=len(m.group(1)) - 1 if len(m.group(1)) > 1 else 0)
        elif img:
            ruta = (md.parent / img.group(2)).resolve()
            if ruta.exists():
                doc.add_picture(str(ruta), width=Cm(15.5))
                if img.group(1):
                    p = doc.add_paragraph(); en_linea(p, f"*{img.group(1)}*", 10)
        elif re.match(r"^\s*[-•]\s+", ln):
            en_linea(doc.add_paragraph(style="List Bullet"), re.sub(r"^\s*[-•]\s+", "", ln))
        elif re.match(r"^\s*\d+\.\s+", ln):
            en_linea(doc.add_paragraph(style="List Number"), re.sub(r"^\s*\d+\.\s+", "", ln))
        elif ln.strip():
            en_linea(doc.add_paragraph(), ln.strip())
        i += 1
    doc.save(salida)


if __name__ == "__main__":
    e = Path(sys.argv[1]).resolve()
    convertir(e, Path(sys.argv[2]) if len(sys.argv) > 2 else e.with_suffix(".docx"))

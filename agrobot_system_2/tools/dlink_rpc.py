"""Cliente mínimo de la API JSON-RPC del router D-Link DIR-615X (firmware anweb).

Uso:
    DL_PASS=<clave admin> python3 dlink_rpc.py POST /jsonrpc '<json>'

o como módulo: `import dlink_rpc; dlink_rpc.req("POST", "/jsonrpc", datos)`.

La autenticación es un digest MD5 con cabecera propia (`anweb-authenticate`)
y una cookie de sesión (`device-session-id`) que hay que devolver. El router
bloquea tras 5 intentos fallidos. Ids útiles y trampas conocidas: ver
docs/RED_E_IPS.md. NO leer ids al azar: algunos disparan acciones (un
barrido de ids reinició la radio el 2026-09-30).
"""
import hashlib, os, sys, urllib.request, urllib.error

HOST = os.environ.get("DL_HOST", "http://192.168.0.1")
USER = os.environ.get("DL_USER", "admin")
PASS = os.environ.get("DL_PASS", "")
md5 = lambda s: hashlib.md5(s.encode()).hexdigest()
_nc = 0
_chal = None
_cookie = {}


def _raw(method, url, data=None, headers=None):
    headers = dict(headers or {})
    if _cookie:
        headers["Cookie"] = "; ".join(f"{k}={v}" for k, v in _cookie.items())
    req = urllib.request.Request(HOST + url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=20) as r:
            st, h, body = r.status, dict(r.headers), r.read()
    except urllib.error.HTTPError as e:
        st, h, body = e.code, dict(e.headers), e.read()
    sc = h.get("Set-Cookie", "")
    if "=" in sc:
        k, _, v = sc.split(";")[0].partition("=")
        _cookie[k.strip()] = v.strip()
    return st, h, body


def _parse(h):
    out = {}
    for part in h.replace("Digest", "").split(","):
        k, _, v = part.strip().partition("=")
        out[k] = v.strip('"')
    return out


def req(method, url, data=None, ctype=None, _retry=True):
    """Petición autenticada. Reintenta una vez si el nonce caducó."""
    global _nc, _chal
    hdr = {"Content-Type": ctype} if ctype else {}
    if _chal is None:
        st, h, body = _raw(method, url, data, hdr)
        if st != 401:
            return st, h, body
        _chal = _parse(h["anweb-authenticate"])
    _nc += 1
    nc = f"{_nc:08d}"
    cnonce = os.urandom(8).hex()
    ha1 = md5(f"{USER}:{_chal['realm']}:{PASS}")
    resp = md5(f"{ha1}:{_chal['nonce']}:{nc}:{cnonce}:{_chal['qop']}:{md5(method + ':' + url)}")
    hdr["Authorization"] = (f'Digest username="{USER}", realm="{_chal["realm"]}", nonce="{_chal["nonce"]}", '
                            f'uri="{url}", response="{resp}", qop={_chal["qop"]}, nc={nc}, cnonce="{cnonce}"')
    st, h, body = _raw(method, url, data, hdr)
    if st == 401 and _retry and "anweb-authenticate" in h:
        _chal = _parse(h["anweb-authenticate"])
        return req(method, url, data, ctype, _retry=False)
    if st == 401:
        sys.stderr.write(f"401 — intentos restantes: {h.get('anweb-auth-try-count-remain')}\n")
    return st, h, body


if __name__ == "__main__":
    method, url = sys.argv[1], sys.argv[2]
    data = sys.argv[3].encode() if len(sys.argv) > 3 else None
    st, h, body = req(method, url, data, "application/json" if data else None)
    sys.stderr.write(f"HTTP {st}\n")
    sys.stdout.buffer.write(body)

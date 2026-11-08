"""Codigo QR (ISO/IEC 18004) para la URL del control desde el celular.

Solo biblioteca estandar, como servidor.py (en la PC no hay qrcode ni
qrencode y el paquete se exporta a otras PC). Modo byte, correccion M,
versiones 1 a 10 (hasta 213 bytes: de sobra para una URL). Sigue el
algoritmo de referencia de Nayuki (qrcodegen, MIT).

    from qr import qr_matriz, qr_terminal
    print(qr_terminal("http://10.0.0.5:8420/"))
"""

# Correccion M. Indice = version (0 sin uso).
_ECC_POR_BLOQUE = [-1, 10, 16, 26, 18, 24, 16, 18, 22, 22, 26]
_BLOQUES = [-1, 1, 1, 1, 2, 2, 4, 4, 4, 5, 5]
_FORMATO_M = 0      # bits de formato de la correccion M
_VERSION_MAX = 10


def _modulos_datos(ver):
    n = (16 * ver + 128) * ver + 64
    if ver >= 2:
        k = ver // 7 + 2
        n -= (25 * k - 10) * k - 55
        if ver >= 7:
            n -= 36
    return n


def _palabras_datos(ver):
    return _modulos_datos(ver) // 8 - _ECC_POR_BLOQUE[ver] * _BLOQUES[ver]


def _gf_mul(x, y):
    z = 0
    for i in reversed(range(8)):
        z = (z << 1) ^ ((z >> 7) * 0x11D)
        z ^= ((y >> i) & 1) * x
    return z


def _divisor(grado):
    r = [0] * (grado - 1) + [1]
    raiz = 1
    for _ in range(grado):
        for j in range(grado):
            r[j] = _gf_mul(r[j], raiz)
            if j + 1 < grado:
                r[j] ^= r[j + 1]
        raiz = _gf_mul(raiz, 0x02)
    return r


def _residuo(datos, divisor):
    r = [0] * len(divisor)
    for b in datos:
        f = b ^ r.pop(0)
        r.append(0)
        for i, c in enumerate(divisor):
            r[i] ^= _gf_mul(c, f)
    return r


def _codificar(texto):
    datos = texto.encode("utf-8")
    for ver in range(1, _VERSION_MAX + 1):
        bits_cuenta = 8 if ver <= 9 else 16
        capacidad = _palabras_datos(ver) * 8
        if 4 + bits_cuenta + 8 * len(datos) <= capacidad:
            break
    else:
        raise ValueError(f"texto demasiado largo para un QR version {_VERSION_MAX}")
    bits = []

    def poner(valor, n):
        bits.extend((valor >> i) & 1 for i in reversed(range(n)))

    poner(0b0100, 4)                     # modo byte
    poner(len(datos), bits_cuenta)
    for b in datos:
        poner(b, 8)
    poner(0, min(4, capacidad - len(bits)))
    poner(0, -len(bits) % 8)
    relleno = 0xEC
    while len(bits) < capacidad:
        poner(relleno, 8)
        relleno ^= 0xEC ^ 0x11
    palabras = [int("".join(map(str, bits[i:i + 8])), 2) for i in range(0, len(bits), 8)]

    # Reed-Solomon por bloque y entrelazado
    nb, ecc = _BLOQUES[ver], _ECC_POR_BLOQUE[ver]
    crudas = _modulos_datos(ver) // 8
    cortos = nb - crudas % nb
    largo_corto = crudas // nb
    div = _divisor(ecc)
    bloques, k = [], 0
    for i in range(nb):
        d = palabras[k:k + largo_corto - ecc + (0 if i < cortos else 1)]
        k += len(d)
        e = _residuo(d, div)
        if i < cortos:
            d.append(0)
        bloques.append(d + e)
    salida = []
    for i in range(len(bloques[0])):
        for j, b in enumerate(bloques):
            if i != largo_corto - ecc or j >= cortos:
                salida.append(b[i])
    return ver, salida


class _Matriz:
    def __init__(self, ver):
        self.ver = ver
        self.n = ver * 4 + 17
        self.m = [[False] * self.n for _ in range(self.n)]
        self.fija = [[False] * self.n for _ in range(self.n)]

    def fijar(self, x, y, oscuro):
        self.m[y][x] = oscuro
        self.fija[y][x] = True

    def patrones(self):
        n = self.n
        for i in range(n):
            self.fijar(6, i, i % 2 == 0)
            self.fijar(i, 6, i % 2 == 0)
        for cx, cy in ((3, 3), (n - 4, 3), (3, n - 4)):
            for dy in range(-4, 5):
                for dx in range(-4, 5):
                    x, y = cx + dx, cy + dy
                    if 0 <= x < n and 0 <= y < n:
                        self.fijar(x, y, max(abs(dx), abs(dy)) not in (2, 4))
        if self.ver > 1:
            k = self.ver // 7 + 2
            paso = (self.ver * 8 + k * 3 + 5) // (k * 4 - 4) * 2
            pos = list(reversed([n - 7 - i * paso for i in range(k - 1)] + [6]))
            for i, a in enumerate(pos):
                for j, b in enumerate(pos):
                    if (i, j) in ((0, 0), (0, k - 1), (k - 1, 0)):
                        continue
                    for dy in range(-2, 3):
                        for dx in range(-2, 3):
                            self.fijar(a + dx, b + dy, max(abs(dx), abs(dy)) != 1)
        self.formato(0)
        if self.ver >= 7:
            r = self.ver
            for _ in range(12):
                r = (r << 1) ^ ((r >> 11) * 0x1F25)
            bits = self.ver << 12 | r
            for i in range(18):
                bit = (bits >> i) & 1 == 1
                a, b = n - 11 + i % 3, i // 3
                self.fijar(a, b, bit)
                self.fijar(b, a, bit)

    def formato(self, mascara):
        datos = _FORMATO_M << 3 | mascara
        r = datos
        for _ in range(10):
            r = (r << 1) ^ ((r >> 9) * 0x537)
        bits = (datos << 10 | r) ^ 0x5412
        bit = lambda i: (bits >> i) & 1 == 1  # noqa: E731
        n = self.n
        for i in range(6):
            self.fijar(8, i, bit(i))
        self.fijar(8, 7, bit(6))
        self.fijar(8, 8, bit(7))
        self.fijar(7, 8, bit(8))
        for i in range(9, 15):
            self.fijar(14 - i, 8, bit(i))
        for i in range(8):
            self.fijar(n - 1 - i, 8, bit(i))
        for i in range(8, 15):
            self.fijar(8, n - 15 + i, bit(i))
        self.fijar(8, n - 8, True)

    def datos(self, palabras):
        n, i, total = self.n, 0, len(palabras) * 8
        derecha = n - 1
        while derecha >= 1:
            if derecha == 6:
                derecha = 5
            for vert in range(n):
                for j in range(2):
                    x = derecha - j
                    sube = (derecha + 1) & 2 == 0
                    y = n - 1 - vert if sube else vert
                    if not self.fija[y][x] and i < total:
                        self.m[y][x] = (palabras[i >> 3] >> (7 - (i & 7))) & 1 == 1
                        i += 1
            derecha -= 2

    def mascara(self, k):
        f = [lambda x, y: (x + y) % 2 == 0, lambda x, y: y % 2 == 0, lambda x, y: x % 3 == 0,
             lambda x, y: (x + y) % 3 == 0, lambda x, y: (x // 3 + y // 2) % 2 == 0,
             lambda x, y: x * y % 2 + x * y % 3 == 0, lambda x, y: (x * y % 2 + x * y % 3) % 2 == 0,
             lambda x, y: ((x + y) % 2 + x * y % 3) % 2 == 0][k]
        for y in range(self.n):
            for x in range(self.n):
                if not self.fija[y][x] and f(x, y):
                    self.m[y][x] = not self.m[y][x]

    def castigo(self):
        n, m, p = self.n, self.m, 0
        filas = m + [list(c) for c in zip(*m)]
        for fila in filas:
            racha = 1
            for a, b in zip(fila, fila[1:]):
                if a == b:
                    racha += 1
                else:
                    if racha >= 5:
                        p += racha - 2
                    racha = 1
            if racha >= 5:
                p += racha - 2
            s = "".join("1" if v else "0" for v in fila)
            p += 40 * (s.count("10111010000") + s.count("00001011101"))
        for y in range(n - 1):
            for x in range(n - 1):
                if m[y][x] == m[y][x + 1] == m[y + 1][x] == m[y + 1][x + 1]:
                    p += 3
        oscuros = sum(map(sum, m))
        p += (abs(oscuros * 20 - n * n * 10) + n * n - 1) // (n * n) * 10 - 10
        return p


def qr_matriz(texto):
    """Lista de filas (True = modulo oscuro), sin zona de silencio."""
    ver, palabras = _codificar(texto)
    mejor = None
    for k in range(8):
        q = _Matriz(ver)
        q.patrones()
        q.datos(palabras)
        q.mascara(k)
        q.formato(k)
        p = q.castigo()
        if mejor is None or p < mejor[0]:
            mejor = (p, q)
    return mejor[1].m


def qr_terminal(texto, margen=2):
    """El QR con medios bloques y colores ANSI (oscuro sobre blanco), para
    que se lea igual en una terminal de fondo negro o blanco."""
    m = qr_matriz(texto)
    n = len(m)
    claro = [[False] * (n + 2 * margen) for _ in range(margen)]
    filas = claro + [[False] * margen + f + [False] * margen for f in m] + claro
    if len(filas) % 2:
        filas.append([False] * len(filas[0]))
    lineas = []
    for y in range(0, len(filas), 2):
        arriba, abajo = filas[y], filas[y + 1]
        s = "".join(("\033[30m" if a else "\033[97m") + ("\033[40m" if b else "\033[107m") + "▀"
                    for a, b in zip(arriba, abajo))
        lineas.append(s + "\033[0m")
    return "\n".join(lineas)


def qr_pgm(texto, ruta, escala=10, margen=4):
    """Guarda el QR como imagen PGM (en escala de grises, sin bibliotecas)."""
    m = qr_matriz(texto)
    n = len(m) + 2 * margen
    with open(ruta, "wb") as f:
        f.write(f"P5 {n * escala} {n * escala} 255\n".encode())
        for y in range(n):
            fila = bytearray()
            for x in range(n):
                xi, yi = x - margen, y - margen
                oscuro = 0 <= xi < len(m) and 0 <= yi < len(m) and m[yi][xi]
                fila += bytes([0 if oscuro else 255]) * escala
            f.write(bytes(fila) * escala)

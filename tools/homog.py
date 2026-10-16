def homography(info):
    W, H = info["W"], info["H"]
    pix = {"tl": (0, 0), "tr": (W, 0), "br": (W, H), "bl": (0, H)}
    pairs = [(info["corners"][k][:2], pix[k]) for k in pix]
    def solve(A, b):
        n = len(A); M = [row[:] + [b[i]] for i, row in enumerate(A)]
        for c in range(n):
            p = max(range(c, n), key=lambda r: abs(M[r][c])); M[c], M[p] = M[p], M[c]
            for r in range(n):
                if r != c:
                    fct = M[r][c] / M[c][c]; M[r] = [M[r][k] - fct * M[c][k] for k in range(n + 1)]
        return [M[i][n] / M[i][i] for i in range(n)]
    A, b = [], []
    for (x, y), (u, v) in pairs:
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.append(u)
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y]); b.append(v)
    h = solve(A, b)
    def P(x, y):
        w = h[6] * x + h[7] * y + 1
        return ((h[0] * x + h[1] * y + h[2]) / w, (h[3] * x + h[4] * y + h[5]) / w)
    return P

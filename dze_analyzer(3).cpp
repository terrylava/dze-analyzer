// DZE Build Collision Analyzer - Win32 native GUI
// ---------------------------------------------------------------------------
// Loads DayZ Editor .dze files, clusters object coordinates into "builds"
// (single-linkage at 50m on X/Z), draws each build as a cylinder over a
// map image, and flags cross-file cylinder collisions.
//
// Build (MSVC):
//   cl /EHsc /O2 dze_analyzer.cpp /link gdiplus.lib comdlg32.lib gdi32.lib user32.lib shell32.lib ole32.lib
//
// Build (MinGW-w64):
//   x86_64-w64-mingw32-g++ -O2 dze_analyzer.cpp -o dze_analyzer.exe ^
//       -lgdiplus -lcomdlg32 -lgdi32 -luser32 -lshell32 -lole32 -static -mwindows
// ---------------------------------------------------------------------------

#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include <commdlg.h>
#include <shlobj.h>
#include <gdiplus.h>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <cctype>
#include <cstdlib>
#include <cstring>

#pragma comment(lib, "gdiplus.lib")
#pragma comment(lib, "comdlg32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "ole32.lib")

using namespace Gdiplus;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
static double g_clusterDist   = 50.0;    // meters, X/Z single-linkage distance
static double g_worldSize     = 15360.0; // Chernarus meters (edit in UI field)
static bool   g_flipZ         = true;    // world origin bottom-left -> image top-left
static double g_elongRatio    = 4.0;     // bbox aspect ratio to flag as elongated

// ---------------------------------------------------------------------------
// Data model
// ---------------------------------------------------------------------------
struct Build {
    double cx = 0, cz = 0;     // centroid
    double radius = 0;         // max distance from centroid to a member
    int    count = 0;          // number of objects
    double bboxW = 0, bboxL = 0; // bounding box dims (sorted so L >= W)
    bool   elongated = false;
};

struct DzeFile {
    std::wstring path;
    std::wstring name;
    std::string  mapName;
    int          version = 0;
    std::vector<std::pair<double,double>> coords; // (x, z)
    std::vector<Build> builds;
    Color        color = Color(255,255,255,255);
    bool         ok = false;
    std::wstring error;
};

struct Collision {
    int fileA, buildA;
    int fileB, buildB;
    double centerDist, radiiSum, overlap, midX, midZ;
};

static std::vector<DzeFile> g_files;
static std::vector<Collision> g_collisions;
static std::wstring g_report;

// View state
static Image*  g_mapImage = nullptr;
static double  g_viewScale = 1.0;   // pixels-per-image-pixel
static double  g_viewX = 0, g_viewY = 0; // top-left of image in client coords
static bool    g_dragging = false;
static POINT   g_dragStart;
static double  g_dragViewX, g_dragViewY;

// Hover state: which build the cursor is over (-1 = none)
static int     g_hoverFile = -1;
static int     g_hoverBuild = -1;
static POINT   g_hoverPt = {0,0}; // client-space cursor for drawing the info box

// Palette for files
static const Color kPalette[] = {
    Color(255, 66,133,244), Color(255,234, 67, 53), Color(255,251,188, 5),
    Color(255, 52,168, 83), Color(255,163, 74,255), Color(255,255,133, 27),
    Color(255, 26,188,156), Color(255,233, 30, 99), Color(255,121, 85, 72),
    Color(255,  0,188,212),
};

// Control IDs
enum {
    ID_ADDFILES = 1001, ID_ADDFOLDER, ID_LOADMAP, ID_WORLDSIZE,
    ID_FLIPZ, ID_ANALYZE, ID_LIST, ID_REPORT, ID_STATUS
};
static HWND g_hMain, g_hCanvas, g_hList, g_hReport, g_hWorldEdit, g_hFlipZ, g_hStatus;

// ---------------------------------------------------------------------------
// .dze decoder (validated layout)
// ---------------------------------------------------------------------------
class DzeReader {
    const std::vector<char>& d;
    size_t p = 0;
public:
    DzeReader(const std::vector<char>& data) : d(data) {}
    bool eof() const { return p >= d.size(); }
    bool need(size_t n) const { return p + n <= d.size(); }

    bool i32(int32_t& out) {
        if (!need(4)) return false;
        std::memcpy(&out, d.data()+p, 4); p += 4; return true;
    }
    bool u32(uint32_t& out) {
        if (!need(4)) return false;
        std::memcpy(&out, d.data()+p, 4); p += 4; return true;
    }
    bool f32(float& out) {
        if (!need(4)) return false;
        std::memcpy(&out, d.data()+p, 4); p += 4; return true;
    }
    bool skip(size_t n) { if (!need(n)) return false; p += n; return true; }

    bool str(std::string& out) {
        uint32_t n;
        if (!u32(n)) return false;
        if (n == 0) { out.clear(); return true; }
        if (n > 65536 || !need(n)) return false;
        out.assign(d.data()+p, n);
        p += n;
        while (!out.empty() && out.back() == '\0') out.pop_back();
        return true;
    }
    bool vec(float& x, float& y, float& z) {
        int32_t c;
        return i32(c) && f32(x) && f32(y) && f32(z);
    }
};

static bool readParam(DzeReader& r) {
    // Parameter entry: key(string), type(string), then a type-sized payload.
    // Observed types are SerializableParam1<bool|int|float> (4-byte scalar).
    // String/vector payloads handled by type-name inspection for safety.
    std::string key, ptype;
    if (!r.str(key)) return false;
    if (!r.str(ptype)) return false;
    // lowercase copy for matching
    std::string low = ptype;
    for (char& c : low) c = (char)tolower((unsigned char)c);
    if (low.find("string") != std::string::npos) {
        std::string val;
        if (!r.str(val)) return false;
    } else if (low.find("vector") != std::string::npos) {
        float x, y, z; if (!r.vec(x, y, z)) return false;
    } else {
        // bool / int / float scalar = 4 bytes
        if (!r.skip(4)) return false;
    }
    return true;
}

static bool parseObject(DzeReader& r, int ver,
                        std::vector<std::pair<double,double>>& coords) {
    std::string type, disp, model;
    if (!r.str(type)) return false;
    if (type.empty()) return false;
    if (!r.str(disp)) return false;

    float px, py, pz, ox, oy, oz, scale;
    int32_t flags;
    if (!r.vec(px, py, pz)) return false;
    if (!r.vec(ox, oy, oz)) return false;
    if (!r.f32(scale)) return false;
    if (!r.i32(flags)) return false;

    if (ver >= 2) {
        // v2..v7 write an Attachments string array here (v8 does not).
        if (ver < 8) {
            int32_t attachCount;
            if (!r.i32(attachCount)) return false;
            if (attachCount < 0 || attachCount > 65536) return false;
            for (int32_t a = 0; a < attachCount; ++a) {
                std::string att;
                if (!r.str(att)) return false;
            }
        }
        // Parameters map (all versions) - count + typed entries.
        // This is populated on objects like NetworkPointLight / NetworkSpotLight.
        int32_t paramCount;
        if (!r.i32(paramCount)) return false;
        if (paramCount < 0 || paramCount > 65536) return false;
        for (int32_t pi = 0; pi < paramCount; ++pi) {
            if (!readParam(r)) return false;
        }
        if (ver >= 3) {
            // 4 NonSerialized bool/reserved words
            int32_t tmp;
            for (int k = 0; k < 4; ++k) if (!r.i32(tmp)) return false;
            if (ver >= 5) {
                if (!r.str(model)) return false;   // Model path (p3d)
            }
            if (ver >= 8) {
                int32_t amc;
                if (!r.i32(amc)) return false;
                if (amc < 0 || amc > 4096) return false;
                for (int32_t a = 0; a < amc; ++a) {
                    int32_t slot;
                    if (!r.i32(slot)) return false;
                    if (!parseObject(r, ver, coords)) return false;
                }
            }
        }
    }
    coords.emplace_back((double)px, (double)pz);
    return true;
}

// Extract object X/Z positions from a JSON-format .dze (non-binned export).
// Minimal hand-rolled scan: finds each "Position": [ x, y, z ] under the
// EditorObjects array. Avoids pulling in a JSON library.
static bool decodeJsonDze(const std::vector<char>& data, DzeFile& f) {
    std::string s(data.begin(), data.end());

    // MapName
    size_t mp = s.find("\"MapName\"");
    if (mp != std::string::npos) {
        size_t q1 = s.find('"', s.find(':', mp) );
        if (q1 != std::string::npos) {
            size_t q2 = s.find('"', q1 + 1);
            if (q2 != std::string::npos) f.mapName = s.substr(q1 + 1, q2 - q1 - 1);
        }
    }

    // Restrict to EditorObjects section if present so we don't pick up
    // DeletedObjects / CameraPosition arrays.
    size_t start = s.find("\"EditorObjects\"");
    if (start == std::string::npos) start = s.find("\"Objects\"");
    size_t end = s.find("\"DeletedObjects\"", start == std::string::npos ? 0 : start);
    if (end == std::string::npos) end = s.size();
    size_t scanFrom = (start == std::string::npos) ? 0 : start;

    size_t pos = scanFrom;
    const std::string key = "\"Position\"";
    while (true) {
        size_t k = s.find(key, pos);
        if (k == std::string::npos || k >= end) break;
        size_t lb = s.find('[', k);
        if (lb == std::string::npos) break;
        // parse three floats
        double vals[3]; int got = 0;
        size_t c = lb + 1;
        while (got < 3 && c < s.size()) {
            // skip non-number chars
            while (c < s.size() && (s[c]==' '||s[c]=='\n'||s[c]=='\r'||s[c]=='\t'||s[c]==',')) c++;
            char* endp = nullptr;
            double v = strtod(s.c_str() + c, &endp);
            if (endp == s.c_str() + c) break;
            vals[got++] = v;
            c = endp - s.c_str();
        }
        if (got == 3) f.coords.emplace_back(vals[0], vals[2]); // X, Z
        pos = c;
    }
    f.version = 0; // JSON
    f.ok = !f.coords.empty();
    if (!f.ok) f.error = L"JSON parsed but no positions found";
    return f.ok;
}

static bool decodeDze(const std::wstring& path, DzeFile& f) {
    std::ifstream in(path, std::ios::binary);
    if (!in) { f.error = L"cannot open file"; return false; }
    std::vector<char> data((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());

    // JSON-format .dze? (editor can export plain JSON instead of binned)
    if (!data.empty()) {
        size_t i = 0;
        while (i < data.size() && (data[i]==' '||data[i]=='\n'||data[i]=='\r'||data[i]=='\t')) i++;
        if (i < data.size() && data[i] == '{') {
            return decodeJsonDze(data, f);
        }
    }

    DzeReader r(data);

    std::string magic;
    if (!r.str(magic) || magic != "EditorBinned") { f.error = L"bad magic"; return false; }
    int32_t ver;
    if (!r.i32(ver)) { f.error = L"truncated header"; return false; }
    f.version = ver;
    std::string mapName;
    if (!r.str(mapName)) { f.error = L"truncated mapname"; return false; }
    f.mapName = mapName;
    float cx, cy, cz;
    if (!r.vec(cx, cy, cz)) { f.error = L"truncated camera"; return false; }

    if (ver >= 7) {
        std::string author;
        if (!r.str(author)) { f.error = L"truncated author"; return false; }
        int32_t cc;
        if (!r.i32(cc)) return false;
        if (cc < 0 || cc > 4096) { f.error = L"bad credit count"; return false; }
        for (int32_t i = 0; i < cc; ++i) { std::string s; if (!r.str(s)) return false; }
        if (!r.skip(8)) { f.error = L"truncated dates"; return false; }
    }

    int32_t oc;
    if (!r.i32(oc)) { f.error = L"truncated object count"; return false; }
    if (oc < 0 || oc > 1000000) { f.error = L"absurd object count"; return false; }

    for (int32_t i = 0; i < oc; ++i) {
        if (r.eof()) break;
        if (!parseObject(r, ver, f.coords)) { f.error = L"object parse desync"; return false; }
    }
    // Hidden/deleted objects (best-effort; not needed for clustering)
    int32_t hc;
    if (r.i32(hc) && hc >= 0 && hc <= 1000000) {
        for (int32_t i = 0; i < hc; ++i) {
            std::string t; float hx, hy, hz; int32_t hf;
            if (!r.str(t) || !r.vec(hx, hy, hz) || !r.i32(hf)) break;
            if (ver >= 8) { std::string m; r.str(m); }
        }
    }
    f.ok = true;
    return true;
}

// ---------------------------------------------------------------------------
// Clustering (single-linkage / union-find) + cylinder + elongation
// ---------------------------------------------------------------------------
static int uf_find(std::vector<int>& parent, int a) {
    while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
    return a;
}

static void computeBuilds(DzeFile& f) {
    f.builds.clear();
    const auto& c = f.coords;
    int n = (int)c.size();
    if (n == 0) return;

    std::vector<int> parent(n);
    for (int i = 0; i < n; ++i) parent[i] = i;
    double d2 = g_clusterDist * g_clusterDist;
    for (int a = 0; a < n; ++a) {
        for (int b = a+1; b < n; ++b) {
            double dx = c[a].first - c[b].first;
            double dz = c[a].second - c[b].second;
            if (dx*dx + dz*dz <= d2) {
                int ra = uf_find(parent, a), rb = uf_find(parent, b);
                if (ra != rb) parent[ra] = rb;
            }
        }
    }
    std::map<int, std::vector<int>> groups;
    for (int i = 0; i < n; ++i) groups[uf_find(parent, i)].push_back(i);

    for (auto& kv : groups) {
        Build bld;
        auto& idx = kv.second;
        bld.count = (int)idx.size();
        double sx = 0, sz = 0;
        double minx = 1e18, maxx = -1e18, minz = 1e18, maxz = -1e18;
        for (int i : idx) {
            sx += c[i].first; sz += c[i].second;
            minx = std::min(minx, c[i].first); maxx = std::max(maxx, c[i].first);
            minz = std::min(minz, c[i].second); maxz = std::max(maxz, c[i].second);
        }
        bld.cx = sx / bld.count;
        bld.cz = sz / bld.count;
        double r = 0;
        for (int i : idx) {
            double dx = c[i].first - bld.cx, dz = c[i].second - bld.cz;
            r = std::max(r, std::sqrt(dx*dx + dz*dz));
        }
        bld.radius = r;
        double w = maxx - minx, l = maxz - minz;
        if (w > l) std::swap(w, l);
        bld.bboxW = w; bld.bboxL = l;
        bld.elongated = (w > 0.01) && (l / w >= g_elongRatio);
        // Degenerate line where w==0 but l large -> also elongated
        if (w <= 0.01 && l > g_clusterDist) bld.elongated = true;
        f.builds.push_back(bld);
    }
}

// ---------------------------------------------------------------------------
// Collision detection (different files only, same map)
// ---------------------------------------------------------------------------
static void computeCollisions() {
    g_collisions.clear();
    int nf = (int)g_files.size();
    for (int a = 0; a < nf; ++a) {
        if (!g_files[a].ok) continue;
        for (int b = a+1; b < nf; ++b) {
            if (!g_files[b].ok) continue;
            if (g_files[a].mapName != g_files[b].mapName) continue;
            for (int ia = 0; ia < (int)g_files[a].builds.size(); ++ia) {
                const Build& A = g_files[a].builds[ia];
                for (int ib = 0; ib < (int)g_files[b].builds.size(); ++ib) {
                    const Build& B = g_files[b].builds[ib];
                    double dx = A.cx - B.cx, dz = A.cz - B.cz;
                    double dist = std::sqrt(dx*dx + dz*dz);
                    if (dist < A.radius + B.radius) {
                        Collision col;
                        col.fileA = a; col.buildA = ia;
                        col.fileB = b; col.buildB = ib;
                        col.centerDist = dist;
                        col.radiiSum = A.radius + B.radius;
                        col.overlap = col.radiiSum - dist;
                        col.midX = (A.cx + B.cx) / 2;
                        col.midZ = (A.cz + B.cz) / 2;
                        g_collisions.push_back(col);
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Report generation
// ---------------------------------------------------------------------------
static std::wstring s2w(const std::string& s) {
    if (s.empty()) return L"";
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), nullptr, 0);
    std::wstring w(n, 0);
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), &w[0], n);
    return w;
}

static void buildReport() {
    std::wstringstream ss;
    ss << L"=== BUILDS PER FILE ===\r\n";
    std::vector<std::string> maps;
    for (auto& f : g_files) {
        if (!f.ok) {
            ss << f.name << L": ERROR - " << f.error << L"\r\n";
            continue;
        }
        maps.push_back(f.mapName);
        ss << f.name << L"  [map=" << s2w(f.mapName) << L" v" << f.version
           << L"]  " << (int)f.coords.size() << L" objects -> "
           << (int)f.builds.size() << L" build(s)\r\n";
        std::vector<int> order(f.builds.size());
        for (int i = 0; i < (int)order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(),
                  [&](int x, int y){ return f.builds[x].count > f.builds[y].count; });
        for (int oi = 0; oi < (int)order.size(); ++oi) {
            const Build& b = f.builds[order[oi]];
            wchar_t line[256];
            swprintf(line, 256,
                L"    build %d: center=(%.1f, %.1f)  radius=%.1fm  objects=%d%s\r\n",
                oi, b.cx, b.cz, b.radius, b.count,
                b.elongated ? L"  [ELONGATED - verify]" : L"");
            ss << line;
        }
    }

    bool multiMap = false;
    for (size_t i = 1; i < maps.size(); ++i)
        if (maps[i] != maps[0]) multiMap = true;
    if (multiMap)
        ss << L"\r\n!! WARNING: files span multiple maps - cross-map collisions ignored.\r\n";

    ss << L"\r\n=== COLLISIONS (different files only) ===\r\n";
    if (g_collisions.empty()) {
        ss << L"No cross-file cylinder collisions detected.\r\n";
    } else {
        for (auto& c : g_collisions) {
            wchar_t line[512];
            swprintf(line, 512,
                L"\r\n  %s [build %d] <-> %s [build %d]\r\n"
                L"    center dist=%.1fm  radii sum=%.1fm  overlap=%.1fm  at ~(%.0f, %.0f)\r\n",
                g_files[c.fileA].name.c_str(), c.buildA,
                g_files[c.fileB].name.c_str(), c.buildB,
                c.centerDist, c.radiiSum, c.overlap, c.midX, c.midZ);
            ss << line;
        }
        wchar_t tail[128];
        swprintf(tail, 128, L"\r\n  Total colliding build pairs: %d\r\n",
                 (int)g_collisions.size());
        ss << tail;
    }
    g_report = ss.str();
    SetWindowTextW(g_hReport, g_report.c_str());
}

// ---------------------------------------------------------------------------
// Analyze pipeline
// ---------------------------------------------------------------------------
static void runAnalysis() {
    // read world size from edit
    wchar_t buf[64];
    GetWindowTextW(g_hWorldEdit, buf, 64);
    double ws = _wtof(buf);
    if (ws > 1.0) g_worldSize = ws;
    g_flipZ = (SendMessageW(g_hFlipZ, BM_GETCHECK, 0, 0) == BST_CHECKED);

    for (auto& f : g_files) {
        if (f.ok) computeBuilds(f);
    }
    computeCollisions();
    buildReport();
    InvalidateRect(g_hCanvas, nullptr, FALSE);

    wchar_t st[128];
    swprintf(st, 128, L"%d file(s), %d collision(s)",
             (int)g_files.size(), (int)g_collisions.size());
    SetWindowTextW(g_hStatus, st);
}

// ---------------------------------------------------------------------------
// File loading
// ---------------------------------------------------------------------------
static void addFile(const std::wstring& path) {
    for (auto& f : g_files) if (f.path == path) return; // dedupe
    DzeFile f;
    f.path = path;
    size_t slash = path.find_last_of(L"\\/");
    f.name = (slash == std::wstring::npos) ? path : path.substr(slash+1);
    decodeDze(path, f);
    f.color = kPalette[g_files.size() % (sizeof(kPalette)/sizeof(kPalette[0]))];
    g_files.push_back(f);

    std::wstring item = f.name + (f.ok ? L"" : (L"  (ERR: " + f.error + L")"));
    SendMessageW(g_hList, LB_ADDSTRING, 0, (LPARAM)item.c_str());
}

static void doAddFiles() {
    wchar_t buf[65536] = {0};
    OPENFILENAMEW ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = g_hMain;
    ofn.lpstrFilter = L"DZE Files\0*.dze\0All Files\0*.*\0";
    ofn.lpstrFile = buf;
    ofn.nMaxFile = 65536;
    ofn.Flags = OFN_ALLOWMULTISELECT | OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
    if (!GetOpenFileNameW(&ofn)) return;

    std::wstring dir = buf;
    wchar_t* p = buf + dir.size() + 1;
    if (*p == 0) {
        addFile(dir); // single selection
    } else {
        while (*p) {
            std::wstring fn = p;
            addFile(dir + L"\\" + fn);
            p += fn.size() + 1;
        }
    }
    runAnalysis();
}

static void scanFolder(const std::wstring& dir) {
    std::wstring pattern = dir + L"\\*.dze";
    WIN32_FIND_DATAW fd;
    HANDLE h = FindFirstFileW(pattern.c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return;
    do {
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
            addFile(dir + L"\\" + fd.cFileName);
    } while (FindNextFileW(h, &fd));
    FindClose(h);
}

static void doAddFolder() {
    BROWSEINFOW bi = {0};
    bi.hwndOwner = g_hMain;
    bi.lpszTitle = L"Select folder containing .dze files";
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    LPITEMIDLIST pidl = SHBrowseForFolderW(&bi);
    if (!pidl) return;
    wchar_t path[MAX_PATH];
    if (SHGetPathFromIDListW(pidl, path)) scanFolder(path);
    CoTaskMemFree(pidl);
    runAnalysis();
}

static void doLoadMap() {
    wchar_t buf[MAX_PATH] = {0};
    OPENFILENAMEW ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = g_hMain;
    ofn.lpstrFilter = L"Images\0*.png;*.jpg;*.jpeg;*.bmp\0All Files\0*.*\0";
    ofn.lpstrFile = buf;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
    if (!GetOpenFileNameW(&ofn)) return;
    if (g_mapImage) { delete g_mapImage; g_mapImage = nullptr; }
    g_mapImage = Image::FromFile(buf);
    if (g_mapImage && g_mapImage->GetLastStatus() != Ok) {
        delete g_mapImage; g_mapImage = nullptr;
        MessageBoxW(g_hMain, L"Failed to load image.", L"Error", MB_ICONERROR);
        return;
    }
    // reset view to fit
    RECT rc; GetClientRect(g_hCanvas, &rc);
    if (g_mapImage) {
        double iw = g_mapImage->GetWidth(), ih = g_mapImage->GetHeight();
        double sx = (rc.right) / iw, sy = (rc.bottom) / ih;
        g_viewScale = std::min(sx, sy);
        g_viewX = (rc.right - iw*g_viewScale)/2;
        g_viewY = (rc.bottom - ih*g_viewScale)/2;
    }
    InvalidateRect(g_hCanvas, nullptr, TRUE);
}

// ---------------------------------------------------------------------------
// world -> image-pixel -> client mapping
// ---------------------------------------------------------------------------
static void worldToClient(double wx, double wz, double& sx, double& sy) {
    double iw = g_mapImage ? g_mapImage->GetWidth()  : g_worldSize;
    double ih = g_mapImage ? g_mapImage->GetHeight() : g_worldSize;
    double px = (wx / g_worldSize) * iw;
    double py = g_flipZ ? (1.0 - wz / g_worldSize) * ih : (wz / g_worldSize) * ih;
    sx = g_viewX + px * g_viewScale;
    sy = g_viewY + py * g_viewScale;
}
static double worldRadiusToClient(double r) {
    double iw = g_mapImage ? g_mapImage->GetWidth() : g_worldSize;
    return (r / g_worldSize) * iw * g_viewScale;
}

// Hit-test a client point against all build cylinders.
// If several overlap, pick the one whose center is nearest the cursor.
// Writes result to g_hoverFile / g_hoverBuild (-1 if none). Returns true if changed.
static bool hitTest(int mx, int my) {
    int bestF = -1, bestB = -1;
    double bestCenterDist = 1e18;
    for (int fi = 0; fi < (int)g_files.size(); ++fi) {
        if (!g_files[fi].ok) continue;
        for (int bi = 0; bi < (int)g_files[fi].builds.size(); ++bi) {
            const Build& b = g_files[fi].builds[bi];
            double sx, sy;
            worldToClient(b.cx, b.cz, sx, sy);
            double sr = worldRadiusToClient(b.radius);
            if (sr < 4) sr = 4; // easier to grab tiny cylinders
            double dx = mx - sx, dy = my - sy;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d <= sr && d < bestCenterDist) {
                bestCenterDist = d; bestF = fi; bestB = bi;
            }
        }
    }
    bool changed = (bestF != g_hoverFile || bestB != g_hoverBuild);
    g_hoverFile = bestF; g_hoverBuild = bestB;
    return changed;
}

// ---------------------------------------------------------------------------
// Canvas painting
// ---------------------------------------------------------------------------
static void paintCanvas(HDC hdc, RECT& rc) {
    Bitmap backbuf(rc.right, rc.bottom, PixelFormat32bppARGB);
    Graphics g(&backbuf);
    g.Clear(Color(255, 24, 26, 30));
    g.SetSmoothingMode(SmoothingModeAntiAlias);

    if (g_mapImage) {
        double iw = g_mapImage->GetWidth(), ih = g_mapImage->GetHeight();
        g.DrawImage(g_mapImage,
            RectF((REAL)g_viewX, (REAL)g_viewY,
                  (REAL)(iw*g_viewScale), (REAL)(ih*g_viewScale)));
    }

    // Build collision membership set for highlight
    auto isColliding = [&](int fi, int bi)->bool {
        for (auto& c : g_collisions)
            if ((c.fileA==fi&&c.buildA==bi)||(c.fileB==fi&&c.buildB==bi)) return true;
        return false;
    };

    for (int fi = 0; fi < (int)g_files.size(); ++fi) {
        DzeFile& f = g_files[fi];
        if (!f.ok) continue;
        for (int bi = 0; bi < (int)f.builds.size(); ++bi) {
            Build& b = f.builds[bi];
            double sx, sy;
            worldToClient(b.cx, b.cz, sx, sy);
            double sr = worldRadiusToClient(b.radius);
            if (sr < 2) sr = 2;

            bool coll = isColliding(fi, bi);
            Color fill(60, f.color.GetR(), f.color.GetG(), f.color.GetB());
            SolidBrush brush(fill);
            g.FillEllipse(&brush, (REAL)(sx-sr), (REAL)(sy-sr), (REAL)(2*sr), (REAL)(2*sr));

            REAL penW = coll ? 3.0f : 1.5f;
            Color outline = coll ? Color(255,255,40,40) : f.color;
            if (b.elongated && !coll) outline = Color(255,255,200,0);
            Pen pen(outline, penW);
            if (b.elongated) {
                REAL dash[] = {4,3};
                pen.SetDashPattern(dash, 2);
            }
            g.DrawEllipse(&pen, (REAL)(sx-sr), (REAL)(sy-sr), (REAL)(2*sr), (REAL)(2*sr));

            // center dot
            SolidBrush dot(f.color);
            g.FillEllipse(&dot, (REAL)(sx-2), (REAL)(sy-2), 4.0f, 4.0f);
        }
    }

    // Hover highlight + info box
    if (g_hoverFile >= 0 && g_hoverFile < (int)g_files.size() &&
        g_hoverBuild >= 0 && g_hoverBuild < (int)g_files[g_hoverFile].builds.size()) {
        DzeFile& hf = g_files[g_hoverFile];
        Build& hb = hf.builds[g_hoverBuild];
        double sx, sy;
        worldToClient(hb.cx, hb.cz, sx, sy);
        double sr = worldRadiusToClient(hb.radius);
        if (sr < 4) sr = 4;

        // white ring to mark the hovered cylinder
        Pen hl(Color(255, 255, 255, 255), 2.0f);
        g.DrawEllipse(&hl, (REAL)(sx-sr-1), (REAL)(sy-sr-1),
                      (REAL)(2*sr+2), (REAL)(2*sr+2));

        // compose info text
        wchar_t txt[512];
        swprintf(txt, 512,
            L"%s\r\nbuild %d  |  %d objects\r\ncenter (%.0f, %.0f)\r\nradius %.1f m%s",
            hf.name.c_str(), g_hoverBuild, hb.count, hb.cx, hb.cz, hb.radius,
            hb.elongated ? L"\r\n[ELONGATED - verify]" : L"");

        FontFamily ff(L"Segoe UI");
        Font font(&ff, 12, FontStyleRegular, UnitPixel);
        RectF layout(0, 0, 400, 200), measured;
        g.MeasureString(txt, -1, &font, layout, &measured);

        REAL bx = (REAL)g_hoverPt.x + 16;
        REAL by = (REAL)g_hoverPt.y + 16;
        REAL bw = measured.Width + 14, bh = measured.Height + 10;
        // keep box on-screen
        if (bx + bw > rc.right)  bx = (REAL)g_hoverPt.x - bw - 16;
        if (by + bh > rc.bottom) by = (REAL)g_hoverPt.y - bh - 16;
        if (bx < 0) bx = 2;
        if (by < 0) by = 2;

        SolidBrush bg(Color(235, 20, 22, 26));
        g.FillRectangle(&bg, bx, by, bw, bh);
        Pen border(hf.color, 1.5f);
        g.DrawRectangle(&border, bx, by, bw, bh);
        SolidBrush white(Color(255, 240, 240, 240));
        g.DrawString(txt, -1, &font, PointF(bx + 7, by + 5), &white);
    }

    Graphics screen(hdc);
    screen.DrawImage(&backbuf, 0, 0);
}

// ---------------------------------------------------------------------------
// Window procs
// ---------------------------------------------------------------------------
static LRESULT CALLBACK CanvasProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_PAINT: {
        PAINTSTRUCT ps; HDC hdc = BeginPaint(hwnd, &ps);
        RECT rc; GetClientRect(hwnd, &rc);
        paintCanvas(hdc, rc);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_ERASEBKGND: return 1;
    case WM_LBUTTONDOWN:
        g_dragging = true; g_dragStart.x = LOWORD(lp); g_dragStart.y = HIWORD(lp);
        g_dragViewX = g_viewX; g_dragViewY = g_viewY;
        SetCapture(hwnd); return 0;
    case WM_MOUSEMOVE: {
        int mx = (short)LOWORD(lp), my = (short)HIWORD(lp);
        if (g_dragging) {
            int dx = mx - g_dragStart.x;
            int dy = my - g_dragStart.y;
            g_viewX = g_dragViewX + dx; g_viewY = g_dragViewY + dy;
            InvalidateRect(hwnd, nullptr, FALSE);
        } else {
            g_hoverPt.x = mx; g_hoverPt.y = my;
            bool changed = hitTest(mx, my);
            // always repaint while a build is hovered so the box follows the cursor
            if (changed || g_hoverFile >= 0)
                InvalidateRect(hwnd, nullptr, FALSE);
            // request WM_MOUSELEAVE so we can clear hover when the cursor exits
            TRACKMOUSEEVENT tme = { sizeof(tme) };
            tme.dwFlags = TME_LEAVE; tme.hwndTrack = hwnd;
            TrackMouseEvent(&tme);
        }
        return 0;
    }
    case WM_MOUSELEAVE:
        if (g_hoverFile >= 0) {
            g_hoverFile = g_hoverBuild = -1;
            InvalidateRect(hwnd, nullptr, FALSE);
        }
        return 0;
    case WM_LBUTTONUP:
        g_dragging = false; ReleaseCapture(); return 0;
    case WM_MOUSEWHEEL: {
        POINT pt; pt.x = LOWORD(lp); pt.y = HIWORD(lp);
        ScreenToClient(hwnd, &pt);
        double old = g_viewScale;
        double factor = (GET_WHEEL_DELTA_WPARAM(wp) > 0) ? 1.15 : 1/1.15;
        g_viewScale *= factor;
        if (g_viewScale < 0.02) g_viewScale = 0.02;
        if (g_viewScale > 40) g_viewScale = 40;
        // zoom toward cursor
        g_viewX = pt.x - (pt.x - g_viewX) * (g_viewScale/old);
        g_viewY = pt.y - (pt.y - g_viewY) * (g_viewScale/old);
        InvalidateRect(hwnd, nullptr, FALSE);
        return 0;
    }
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

static LRESULT CALLBACK MainProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_CREATE: {
        HINSTANCE hi = ((LPCREATESTRUCT)lp)->hInstance;

        CreateWindowW(L"BUTTON", L"Add Files...", WS_CHILD|WS_VISIBLE,
            10, 10, 100, 26, hwnd, (HMENU)ID_ADDFILES, hi, nullptr);
        CreateWindowW(L"BUTTON", L"Add Folder...", WS_CHILD|WS_VISIBLE,
            118, 10, 104, 26, hwnd, (HMENU)ID_ADDFOLDER, hi, nullptr);
        CreateWindowW(L"BUTTON", L"Load Map...", WS_CHILD|WS_VISIBLE,
            230, 10, 96, 26, hwnd, (HMENU)ID_LOADMAP, hi, nullptr);

        CreateWindowW(L"STATIC", L"World size (m):", WS_CHILD|WS_VISIBLE,
            10, 46, 100, 20, hwnd, nullptr, hi, nullptr);
        g_hWorldEdit = CreateWindowW(L"EDIT", L"15360",
            WS_CHILD|WS_VISIBLE|WS_BORDER|ES_NUMBER,
            112, 44, 80, 22, hwnd, (HMENU)ID_WORLDSIZE, hi, nullptr);
        g_hFlipZ = CreateWindowW(L"BUTTON", L"Flip Z",
            WS_CHILD|WS_VISIBLE|BS_AUTOCHECKBOX,
            200, 44, 70, 22, hwnd, (HMENU)ID_FLIPZ, hi, nullptr);
        SendMessageW(g_hFlipZ, BM_SETCHECK, BST_CHECKED, 0);
        CreateWindowW(L"BUTTON", L"Analyze", WS_CHILD|WS_VISIBLE,
            278, 43, 80, 24, hwnd, (HMENU)ID_ANALYZE, hi, nullptr);

        g_hList = CreateWindowW(L"LISTBOX", nullptr,
            WS_CHILD|WS_VISIBLE|WS_BORDER|WS_VSCROLL|LBS_NOTIFY,
            10, 78, 348, 150, hwnd, (HMENU)ID_LIST, hi, nullptr);

        g_hReport = CreateWindowW(L"EDIT", nullptr,
            WS_CHILD|WS_VISIBLE|WS_BORDER|WS_VSCROLL|WS_HSCROLL|
            ES_MULTILINE|ES_READONLY|ES_AUTOVSCROLL,
            10, 236, 348, 300, hwnd, (HMENU)ID_REPORT, hi, nullptr);
        HFONT mono = CreateFontW(14,0,0,0,FW_NORMAL,0,0,0,DEFAULT_CHARSET,
            0,0,0,FIXED_PITCH|FF_MODERN, L"Consolas");
        SendMessageW(g_hReport, WM_SETFONT, (WPARAM)mono, TRUE);

        g_hStatus = CreateWindowW(L"STATIC", L"Ready",
            WS_CHILD|WS_VISIBLE|SS_SUNKEN,
            10, 544, 348, 20, hwnd, (HMENU)ID_STATUS, hi, nullptr);

        WNDCLASSW cc = {0};
        cc.lpfnWndProc = CanvasProc;
        cc.hInstance = hi;
        cc.hCursor = LoadCursor(nullptr, IDC_CROSS);
        cc.lpszClassName = L"DzeCanvas";
        RegisterClassW(&cc);
        g_hCanvas = CreateWindowW(L"DzeCanvas", nullptr,
            WS_CHILD|WS_VISIBLE|WS_BORDER,
            370, 10, 600, 554, hwnd, (HMENU)ID_LIST, hi, nullptr);
        return 0;
    }
    case WM_SIZE: {
        RECT rc; GetClientRect(hwnd, &rc);
        int panelW = 360;
        MoveWindow(g_hList,   10, 78, panelW-12, 150, TRUE);
        int repH = rc.bottom - 236 - 30;
        if (repH < 60) repH = 60;
        MoveWindow(g_hReport, 10, 236, panelW-12, repH, TRUE);
        MoveWindow(g_hStatus, 10, rc.bottom-24, panelW-12, 20, TRUE);
        MoveWindow(g_hCanvas, panelW+10, 10,
                   rc.right - panelW - 20, rc.bottom - 20, TRUE);
        return 0;
    }
    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case ID_ADDFILES:  doAddFiles();  break;
        case ID_ADDFOLDER: doAddFolder(); break;
        case ID_LOADMAP:   doLoadMap();   break;
        case ID_ANALYZE:   runAnalysis(); break;
        }
        return 0;
    case WM_DESTROY:
        if (g_mapImage) delete g_mapImage;
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

// ---------------------------------------------------------------------------
int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE, PWSTR, int nShow) {
    CoInitialize(nullptr);
    GdiplusStartupInput gsi;
    ULONG_PTR token;
    GdiplusStartup(&token, &gsi, nullptr);

    WNDCLASSW wc = {0};
    wc.lpfnWndProc = MainProc;
    wc.hInstance = hInst;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE+1);
    wc.lpszClassName = L"DzeAnalyzerMain";
    RegisterClassW(&wc);

    g_hMain = CreateWindowW(L"DzeAnalyzerMain",
        L"DZE Build Collision Analyzer",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, 1000, 620,
        nullptr, nullptr, hInst, nullptr);
    ShowWindow(g_hMain, nShow);
    UpdateWindow(g_hMain);

    MSG msg;
    while (GetMessageW(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    GdiplusShutdown(token);
    CoUninitialize();
    return 0;
}

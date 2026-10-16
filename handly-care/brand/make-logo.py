from fontTools.ttLib import TTFont
from fontTools.pens.svgPathPen import SVGPathPen
from fontTools.pens.transformPen import TransformPen
from fontTools.pens.boundsPen import BoundsPen

def text_path(fontfile, text, size, tracking_em=0.0, x0=0, baseline=0):
    f = TTFont(fontfile); gs = f.getGlyphSet(); cmap = f.getBestCmap(); upm = f['head'].unitsPerEm
    s = size/upm; x = x0; parts=[]
    for ch in text:
        gn = cmap[ord(ch)]; g = gs[gn]
        pen = SVGPathPen(gs)
        tp = TransformPen(pen, (s,0,0,-s,x,baseline))
        g.draw(tp); parts.append(pen.getCommands())
        x += g.width*s + tracking_em*size
    return ' '.join(parts), x - tracking_em*size

SB='/usr/share/fonts/opentype/inter/InterDisplay-SemiBold.otf'
RG='/usr/share/fonts/opentype/inter/InterDisplay-Regular.otf'

# Icon: rounded square, teal->blue gradient, white "h" whose arm offers a dot
def icon(size=100, uid='a'):
    return f'''<defs><linearGradient id="g{uid}" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#3FD0E6"/><stop offset="1" stop-color="#0A7CFF"/></linearGradient>
<linearGradient id="s{uid}" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#fff" stop-opacity=".22"/><stop offset=".5" stop-color="#fff" stop-opacity="0"/></linearGradient></defs>
<rect width="100" height="100" rx="23" fill="url(#g{uid})"/>
<rect width="100" height="100" rx="23" fill="url(#s{uid})"/>
<g fill="none" stroke="#fff" stroke-width="10" stroke-linecap="round" stroke-linejoin="round">
<path d="M33 22V78"/>
<path d="M33 56C33 45.5 40.5 40 49.5 40C59 40 66 45.5 66 56V78"/>
</g>
<circle cx="72" cy="25" r="7.5" fill="#fff"/>'''

W = {}
hp, hw = text_path(SB,'Handly',64,-0.022,0,0)
cp, cw = text_path(RG,'Care',64,-0.018,0,0)
gap = 64*0.24
def wordmark(c1, c2, x, base):
    a,aw = text_path(SB,'Handly',64,-0.022,x,base)
    b,bw = text_path(RG,'Care',64,-0.018,aw+gap,base)
    return f'<path d="{a}" fill="{c1}"/><path d="{b}" fill="{c2}"/>', bw

def lockup(theme):
    c1,c2 = ('#1D1D1F','#6E6E73') if theme=='light' else ('#F5F5F7','#A1A1A6')
    # icon 100 -> scaled to 88, text cap-height aligned
    wm, end = wordmark(c1,c2, 112, 71)
    w = int(end+6)
    bg = '' 
    return f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} 100" width="{w*2}" height="200"><g transform="translate(0,6) scale(.88)">{icon(uid=theme)}</g>{wm}</svg>'

def wordmark_only(theme):
    c1,c2 = ('#1D1D1F','#6E6E73') if theme=='light' else ('#F5F5F7','#A1A1A6')
    wm,end = wordmark(c1,c2,0,64)
    return f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {int(end+2)} 82" width="{int(end+2)*2}" height="164">{wm}</svg>'

open('handly-care-icon.svg','w').write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="512" height="512">{icon()}</svg>')
open('handly-care-logo-light.svg','w').write(lockup('light'))
open('handly-care-logo-dark.svg','w').write(lockup('dark'))
open('handly-care-wordmark-light.svg','w').write(wordmark_only('light'))
open('handly-care-wordmark-dark.svg','w').write(wordmark_only('dark'))
# inline icon snippet for the dashboard (no width/height)
open('icon-inline.svg','w').write(f'<svg viewBox="0 0 100 100" aria-hidden="true">{icon(uid="ui")}</svg>')
print('ok')

import pathlib, re
W=pathlib.Path('web'); css=(W/'style.css').read_text(); body=(W/'body.html').read_text(); ui=(W/'ui.js').read_text()
mock=pathlib.Path('mock-live.js').read_text()
note='<div class="previewnote"><b>Preview with a simulated arm.</b> Press Connect with the fields empty. The downloadable project connects to your real Viam arm.</div>'
art=('<title>Handly Care</title>\n<meta name="claude:viewport-fit-sides" content="cover">\n<style>\n'+css+'</style>\n'
     + body.replace('<!--PREVIEW_NOTE-->',note) + '<script>\n(function(){\n'+mock+'\n'+ui+'\n})();\n</script>\n')
pathlib.Path('arm-control-center.html').write_text(art)
P=pathlib.Path('bedside-arm-live')
(P/'index.html').write_text('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n<title>Handly Care</title>\n<link rel="icon" href="/favicon.svg" type="image/svg+xml">\n<style>\nbody{margin:0}[hidden]{display:none!important}\n'+css+'</style>\n</head>\n<body>\n'+body.replace('<!--PREVIEW_NOTE-->','')+'<script type="module" src="/src/dashboard.js"></script>\n</body>\n</html>\n')
(P/'src/dashboard.js').write_text("import { live, POSES } from './live.js';\n\n"+ui)
open('c.js','w').write(re.search(r'<script>(.*)</script>',art,re.S).group(1))
print('built')

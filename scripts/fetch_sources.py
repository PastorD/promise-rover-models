"""Fetch the public NASA donor assets and check pinned SHA-256 before extraction."""
import urllib.request,hashlib,pathlib,zipfile,json
ROOT=pathlib.Path(__file__).resolve().parents[1];dest=ROOT/'source_assets';dest.mkdir(exist_ok=True)
URLS={
 'curiosity_clean.zip':'https://assets.science.nasa.gov/content/dam/science/cds/3d/resources/model/curiosity-rover-(msl)/Curiosity%20Rover%20(MSL)%20(Clean).zip',
 'perseverance_blend.zip':'https://assets.science.nasa.gov/content/dam/science/cds/3d/resources/model/mars-2020-perseverance-rover/Mars%202020%20Perseverance%20Rover.zip',
 'perseverance.glb':'https://assets.science.nasa.gov/content/dam/science/cds/3d/resources/model/mars-2020-perseverance-rover/Mars%202020%20Perseverance%20Rover.glb',
 'promise_nasa.glb':'https://www.nasa.gov/moonbase/promise-model/promise-rover-draco-x2.glb',
 'os1_rev8_cad.zip':'https://data.ouster.io/downloads/sensor-cad-models/sensor-cad-rev8-os1.zip',
 'os1_rev8_datasheet.pdf':'https://data.ouster.io/downloads/datasheets/datasheet-rev8-v4p0-os1.pdf'}
hashes={x['file']:x['sha256'] for x in json.loads((ROOT/'references/source_checksums.json').read_text())}
for name,url in URLS.items():
    p=dest/name
    if not p.exists():
        req=urllib.request.Request(url,headers={'User-Agent':'Mozilla/5.0','Referer':'https://ouster.com/downloads' if 'ouster' in url else 'https://www.nasa.gov/'})
        p.write_bytes(urllib.request.urlopen(req,timeout=120).read())
    if hashlib.sha256(p.read_bytes()).hexdigest()!=hashes[name]:raise RuntimeError(f'Source asset changed: {name}. Review the new source before rebuilding.')
    if name.endswith('.zip'):
        with zipfile.ZipFile(p) as z:
            target=dest/name[:-4];target.mkdir(exist_ok=True)
            for item in z.infolist():
                if pathlib.PurePosixPath(item.filename).is_absolute() or '..' in pathlib.PurePosixPath(item.filename).parts:raise ValueError('Unsafe zip member')
            z.extractall(target)
    print('Verified',name)

#!/usr/bin/env python3
"""Build the static character assets using the original native MuJoCo renderer."""
from pathlib import Path
import argparse, hashlib, json, os, shutil, struct, subprocess, sys, tempfile

SOURCE = "b50f81255756a9c8bd55dddd2ea72c54d58ebb7d"
NAMES = ("periodic", "age", "uncertainty")
OUTPUT = Path(__file__).resolve().parents[1] / "portraits" / "native"

def valid(path, shape):
    try:
        with path.open("rb") as f:
            header=f.read(33)
        return header[:8]==b"\x89PNG\r\n\x1a\n" and struct.unpack(">II",header[16:24])==shape and header[25]==6
    except (OSError,struct.error,IndexError):
        return False

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("research_root",type=Path)
    parser.add_argument("--reuse-cache",action="store_true",help="Reuse matching versioned local demo PNG files")
    args=parser.parse_args()
    root=args.research_root.resolve()
    generator=root/"demos"/"ai_day_2026"/"generate_portraits.py"
    expected=subprocess.check_output(["git","-C",str(root),"show",SOURCE+":demos/ai_day_2026/generate_portraits.py"])
    if generator.read_bytes()!=expected:
        raise SystemExit("The native generator differs from the pinned source. Use the pinned demo checkout.")
    OUTPUT.mkdir(parents=True,exist_ok=True)
    manifest=OUTPUT/"manifest.json"
    manifest.unlink(missing_ok=True)
    cache=Path(tempfile.gettempdir())/"ai-day-g1-portraits-v8"
    records={}
    for name in NAMES:
        for suffix,shape,extra in [
            (".png",(360,480),("", "", name)),
            ("-motion.png",(2160,4800),(name,)),
            ("-defeat-motion-v14.png",(2160,4320),(name,"defeat")),
        ]:
            filename=name+suffix
            target=OUTPUT/filename
            if args.reuse_cache and valid(cache/filename,shape):
                shutil.copy2(cache/filename,target)
            if not valid(target,shape):
                print("Rendering",filename,flush=True)
                subprocess.run([sys.executable,str(generator),str(OUTPUT),*extra],cwd=root,check=True)
            if not valid(target,shape):
                raise RuntimeError("Invalid or missing portrait: "+filename)
            records[filename]={"sha256":hashlib.sha256(target.read_bytes()).hexdigest(),"width":shape[0],"height":shape[1],"bytes":target.stat().st_size}
    manifest.write_text(json.dumps({"version":1,"source_commit":SOURCE,"assets":records},indent=2)+"\n")
    print(f"Verified {len(records)} native portrait assets ({sum(x['bytes'] for x in records.values()):,} bytes)",flush=True)

if __name__=="__main__":
    main()

"""Native component compatibility matrix; no replacement tracker is implemented."""
import argparse, ctypes as ct, datetime as dt, hashlib, html, itertools, json, os
import platform, subprocess, sys, threading, time
from pathlib import Path
import cv2
import numpy as np
from capture_mock.build import monado_identity, sha
from experiment import MONADO_SOURCE_DIR, source_hashes as experiment_hashes, snapshot_sources
from .observer import make_observer
ROOT=Path(__file__).resolve().parents[1]
DIRECTORY=ROOT/"pipeline_matrix"
BUILD=ROOT/"build"/f"pipeline-matrix-{sys.platform}-{platform.machine()}"
LIBRARY=BUILD/("libwrl_pipeline_matrix.dylib" if sys.platform=="darwin" else "libwrl_pipeline_matrix.so")
UPSTREAM=("u_sink_converter.c","u_sink_force_genlock.c","u_frame.c","u_format.c","u_sink_queue.c","u_sink.h","u_frame.h","u_format.h","u_logging.h")
class View(ct.Structure):
    _fields_=([(x,ct.c_uint32) for x in ("route","format","width","height")]+
              [(x,ct.c_uint64) for x in ("stride","size","sequence","source_id")]+
              [(x,ct.c_int64) for x in ("timestamp","source_timestamp")]+
              [("arrival_ns",ct.c_uint64),("data",ct.POINTER(ct.c_uint8))])
class Stats(ct.Structure):
    _fields_=[(x,ct.c_uint64) for x in ("sent","converted","received","processed","rejected","live","peak","retained_valid","errors","warnings")]
CALLBACK=ct.CFUNCTYPE(ct.c_int,ct.c_void_p,ct.POINTER(View))
def clock_ns():return time.clock_gettime_ns(time.CLOCK_MONOTONIC)
def save(path,data):path.write_text(json.dumps(data,indent=2,allow_nan=False)+"\n")
def source_hashes():
    return experiment_hashes()
def build():
    identity=monado_identity()
    bin_dir=Path(sys.executable).parent
    commands=[
        [str(bin_dir/"cmake"),"-S",str(DIRECTORY/"native"),"-B",str(BUILD),"-G","Ninja",
         f"-DCMAKE_MAKE_PROGRAM={bin_dir/'ninja'}","-DCMAKE_BUILD_TYPE=RelWithDebInfo", f"-DMONADO_SOURCE_DIR={MONADO_SOURCE_DIR}"],
        [str(bin_dir/"cmake"),"--build",str(BUILD),"--parallel","2"],
        [str(bin_dir/"ctest"),"--test-dir",str(BUILD),"--output-on-failure"]]
    logs=[]
    for command in commands:
        r=subprocess.run(command,cwd=ROOT,text=True,capture_output=True,timeout=120)
        logs.append({"command":command,"exit":r.returncode,"output":r.stdout+r.stderr})
        if r.returncode:raise RuntimeError(logs[-1]["output"])
    save(BUILD/"build-record.json",{"monado":identity,"commands":logs,"role":"Native components, not full Monado runtime"})
def library():
    lib=ct.CDLL(str(LIBRARY))
    lib.matrix_view_size.restype=ct.c_uint32
    lib.matrix_stats_size.restype=ct.c_uint32
    if lib.matrix_view_size()!=ct.sizeof(View) or lib.matrix_stats_size()!=ct.sizeof(Stats):
        raise RuntimeError("Rebuild: native ABI size mismatch")
    lib.matrix_create.argtypes=[ct.c_uint32]*4+[CALLBACK,ct.c_void_p];lib.matrix_create.restype=ct.c_void_p
    lib.matrix_push.argtypes=[ct.c_void_p]+[ct.c_uint32]*4+[ct.c_uint64,ct.POINTER(ct.c_uint8),ct.c_uint64,ct.c_uint64,ct.c_int64,ct.c_int64]
    lib.matrix_push.restype=ct.c_int
    lib.matrix_finish.argtypes=[ct.c_void_p,ct.c_uint32,ct.c_uint32];lib.matrix_finish.restype=ct.c_int
    lib.matrix_stats.argtypes=[ct.c_void_p,ct.POINTER(Stats)];lib.matrix_stats.restype=None
    lib.matrix_close.argtypes=[ct.c_void_p];lib.matrix_close.restype=None
    lib.matrix_format.argtypes=[ct.c_uint32];lib.matrix_format.restype=ct.c_uint32
    return lib
def cases():
    plan=json.loads((DIRECTORY/"plan.json").read_text())
    result=[]
    axes=plan["axes"]
    for values in itertools.product(*axes.values()):
        c={**plan["defaults"],**dict(zip(axes,values)),"kind":"format-route"}
        result.append(c)
    o=plan["overload"]
    for fmt,n in itertools.product(o["input_format"],o["cameras"]):
        result.append({**plan["defaults"],**o,"input_format":fmt,"cameras":n,"kind":"overload"})
    p=plan["pairing"]
    for n,order,skew in itertools.product(p["cameras"],p["input_order"],p["first_pair_skew_ns"]):
        result.append({**plan["defaults"],**p,"cameras":n,"input_order":order,
                       "first_pair_skew_ns":skew,"pairing":True,"kind":"pairing"})
    return result
def fixture(config,camera,sequence):
    # Fixture pixels are independent from the upstream conversion implementation.
    h,w=config["height"],config["width"]
    yy,xx=np.indices((h,w))
    gray=np.where((xx//16+yy//16)%2,200,40).astype(np.uint8)
    gray=np.minimum(gray.astype(np.uint16)+camera*5,255).astype(np.uint8)
    x=32+((sequence*3+camera*7+config["seed"])%(w-48))
    gray[h//2:h//2+12,x:x+12]=250
    gray[0,:16]=[87,82,76,1,camera,*sequence.to_bytes(4,"little"),config["seed"],0,1,0,0,0,0]
    return gray
def acceptance(config):
    return config["converter"]=="none" or config["converter"]==config["input_format"] or (config["input_format"]=="L8" and config["converter"]=="RGB")
def execute(config,output,lib=None):
    lib=lib or library()
    cv2.setNumThreads(1)
    native={name:lib.matrix_format(i) for i,name in enumerate(("R8","L8","RGB"))}
    accepted=acceptance(config)
    total=config["cameras"]*config["frames"]
    output.mkdir(parents=True,exist_ok=False)
    save(output/"config.json",config)
    expected_pixels={(cam,seq):fixture(config,cam,seq) for cam in range(config["cameras"]) for seq in range(config["frames"])}
    epoch=clock_ns()
    receive,records,errors=make_observer(config,output,native,expected_pixels,epoch)
    cb=CALLBACK(receive)
    handle=lib.matrix_create(config["cameras"],{"none":0,"L8":1,"RGB":2}[config["converter"]],
                             config["queue_capacity"],int(config["pairing"]),cb,None)
    if not handle:raise RuntimeError("Native matrix graph creation failed")
    stats=Stats()
    def current():
        value=Stats();lib.matrix_stats(handle,ct.byref(value));return value
    try:
        order=list(range(config["cameras"]))
        if config["input_order"]=="reverse":order.reverse()
        for seq in range(config["frames"]):
            for camera in order:
                gray=expected_pixels[(camera,seq)]
                image=np.repeat(gray[:,:,None],3,axis=2).reshape(config["height"],config["width"]*3) if config["input_format"]=="RGB" else gray
                row_size=image.shape[1];stride=row_size+config["padding"]
                padded=np.full((config["height"],stride),165,dtype=np.uint8);padded[:,:row_size]=image
                data=padded.tobytes();buffer=(ct.c_uint8*len(data)).from_buffer_copy(data)
                synthetic=seq*1_000_000_000*config["virtual_fps_den"]//config["virtual_fps_num"]
                skew=config["first_pair_skew_ns"] if seq==0 and camera%2 else 0
                status=lib.matrix_push(handle,camera,native[config["input_format"]],config["width"],
                                       config["height"],stride,buffer,len(data),seq,epoch+synthetic+skew,synthetic+skew)
                if status:raise RuntimeError("Native input validation rejected a valid fixture")
            if config["pairing"]:
                target=config["cameras"]*(seq+1)
                bad_first=config["first_pair_skew_ns"]>1_000_000
                if bad_first:target-=config["cameras"]
                deadline=time.monotonic()+1
                if seq==0 and bad_first:
                    while current().warnings<config["cameras"]//2 and time.monotonic()<deadline:time.sleep(.001)
                    assert current().warnings>=config["cameras"]//2,"Pair skew was not observed"
                else:
                    while current().received<target and time.monotonic()<deadline:time.sleep(.001)
        finish=lib.matrix_finish(handle,total if accepted else 0,config["timeout_ms"])
        lib.matrix_stats(handle,ct.byref(stats))
    finally:lib.matrix_close(handle)
    values={key:int(getattr(stats,key)) for key,_ in Stats._fields_}
    expected_received=total
    if config["pairing"] and config["first_pair_skew_ns"]>1_000_000:expected_received-=config["cameras"]
    checks={"finish":finish==0,"no_callback_error":not errors,"buffers_released":values["live"]==0,
            "retained_pixels_unchanged":values["retained_valid"]==1,"sent":values["sent"]==total,
            "no_duplicates":len({(r["camera_id"],r["sequence"]) for r in records})==len(records),
            "cv_accounted":values["processed"]==len(records) and values["rejected"]==0,
            "converted":values["converted"]==(total if accepted else 0)}
    if not accepted:
        checks["unsupported_format_reported"]=values["errors"]>=total and values["received"]==0
        verdict="FORMAT_BLOCKED"
    elif config["kind"]=="overload":
        checks["overload_accounted"]=0<values["received"]<total
        verdict="WORKS_WITH_DROPS"
    else:
        checks["received"]=values["received"]==expected_received
        verdict="WORKS" if expected_received==total else "WORKS_AFTER_SKEW_REJECTION"
    if config["pairing"]:
        for pair in range(config["cameras"]//2):
            routes=[r["camera_id"] for r in records if r["camera_id"]//2==pair]
            checks[f"pair_{pair}_left_then_right"]=routes==[x for _ in range(expected_received//config["cameras"]) for x in (2*pair,2*pair+1)]
    durations=[r["receive_to_cv_ns"]/1e6 for r in records if "receive_to_cv_ns" in r]
    values["receive_to_cv_median_ms"]=float(np.median(durations)) if durations else None
    result={"config":config,"component_verdict":verdict,"expected_outcome_met":all(checks.values()),
            "counts":values,"unreceived_after_conversion":values["converted"]-values["received"],
            "checks":checks,"errors":errors,"receiver_coverage":"Public diagnostic CV endpoint, not a tracker",
            "clock":"CLOCK_MONOTONIC; source/published times are synthetic","output":str(output)}
    (output/"frames.jsonl").write_text("".join(json.dumps(r)+"\n" for r in records))
    save(output/"summary.json",result)
    return result
def run(output=None):
    build()
    stamp=dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output=Path(output or ROOT/"results/pipeline-matrix"/stamp).resolve()
    output.mkdir(parents=True,exist_ok=False)
    plan=cases()
    hashes=source_hashes()
    manifest={"schema_version":1,"evidence_role":"Measured native converter/queue/pairing components with synthetic after-Capture inputs",
              "created_at_utc":stamp,"cases":plan,"source_hashes":hashes,"seed":1,
              "monado":monado_identity(),"library_sha256":sha(LIBRARY),"python":sys.version,
              "opencv":cv2.__version__,"numpy":np.__version__,"command":[sys.executable,*sys.argv],
              "clock":"CLOCK_MONOTONIC","timing":"Native receiver arrival to CV completion, includes FFI/GIL, validation and injected delay",
              "skipped":["Capture startup (tested separately)","full SLAM/Mercury/Builder","Qualcomm stack","boards"],
              "logging":"New diagnostic stderr transport; converter/queue/genlock/frame/format code unchanged",
              "status":"prepared","build":json.loads((BUILD/"build-record.json").read_text())}
    snapshot_sources(output/"source_snapshot", hashes)
    save(output/"manifest.json",manifest)
    results=[]
    saved_stderr=os.dup(2)
    try:
        with (output/"native-diagnostics.log").open("w") as logs:
            os.dup2(logs.fileno(),2)
            lib=library()
            for index,config in enumerate(plan):
                result=execute(config,output/f"case-{index:03d}",lib)
                results.append(result)
                if not result["expected_outcome_met"]:print(json.dumps({"case":index,"failed":result["checks"]}),flush=True)
    finally:
        os.dup2(saved_stderr,2);os.close(saved_stderr)
    save(output/"summary.json",results)
    rows=[]
    for i,r in enumerate(results):
        c=r["config"];n=r["counts"]
        rows.append(f'<tr><td>{i}</td><td>{c["kind"]}</td><td>{c["input_format"]} → {c["converter"]}</td><td>{c["cameras"]}</td><td>{c["padding"]}</td><td>{c["input_order"]}</td><td>{html.escape(r["component_verdict"])}</td><td>{n["sent"]}/{n["converted"]}/{n["processed"]}</td><td><a href="case-{i:03d}/summary.json">{"expected" if r["expected_outcome_met"] else "UNEXPECTED"}</a></td></tr>')
    report="""<!doctype html><meta charset="utf-8"><title>Native pipeline compatibility matrix</title>
<style>body{font:16px system-ui;margin:32px auto;max-width:1300px;padding:0 20px}table{border-collapse:collapse;width:100%}td,th{padding:9px;border-bottom:1px solid #ddd;text-align:left}aside{padding:18px;background:#fff1cf}</style>
<h1>Which native image paths work?</h1><aside>Measured local component tests. Frames enter after Capture and pass through unchanged Monado converters, queues and optional stereo pairing to a new CV observer. Full trackers, Builder, Qualcomm and boards are untested.</aside>
<p>FORMAT_BLOCKED is a measured incompatibility, not an unexpected test failure. The matrix covers a finite declared set. Source times are synthetic. Stereo pairing averages published timestamps; it does not establish synchronized exposures.</p>
<p><a href="manifest.json">Plan and source manifest</a> · <a href="native-diagnostics.log">Native diagnostics</a> · <a href="summary.json">All measurements</a> · <a href="tracker-readiness.json">Tracker requirements — source inspection only</a></p>
<table><tr><th>Case</th><th>Group</th><th>Input → converter</th><th>Cameras</th><th>Padding</th><th>Feed order</th><th>Measured result</th><th>Sent/converted/CV</th><th>Checks</th></tr>"""+"".join(rows)+"</table>"
    (output/"index.html").write_text(report)
    save(output/"tracker-readiness.json",{
        "evidence_role":"Source inspection; trackers not executed",
        "SLAM":{"source":"t_tracker_slam.cpp","accepted_formats":["L8","RGB"],"needs":["External VIT system library","Configuration/calibration","Backend IMU policy","Camera-0 initial ordering"],"execution":"untested"},
        "Mercury":{"source":"ht_driver.c and t_hand_tracking_async.c","views":2,"needs":["Stereo calibration","Models/runtime","Left-before-right delivery"],"execution":"untested"},
        "image_only_bundle":{"source":"capture_mock/native/mock_capture.cpp","needs":["No requested IMU/pose/mask streams"],"execution":"Previously measured native API tests; not a tracker result"}})
    manifest["status"]="complete"
    manifest["artifact_hashes"]={str(p.relative_to(output)):sha(p) for p in output.rglob("*") if p.is_file() and p!=output/"manifest.json"}
    save(output/"manifest.json",manifest)
    totals={}
    for r in results:totals[r["component_verdict"]]=totals.get(r["component_verdict"],0)+1
    print(json.dumps({"overview":str(output/"index.html"),"cases":len(results),"outcomes":totals,
                      "all_expected":all(r["expected_outcome_met"] for r in results)}),flush=True)
    return all(r["expected_outcome_met"] for r in results)
def main():
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path);parser.add_argument("--build-only",action="store_true")
    args=parser.parse_args()
    if args.build_only:build();return 0
    return 0 if run(args.output) else 1
if __name__=="__main__":raise SystemExit(main())

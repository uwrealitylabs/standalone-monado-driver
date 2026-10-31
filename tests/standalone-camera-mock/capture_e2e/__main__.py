"""Native mock Capture -> original Monado components -> shared public-frame CV."""
import argparse,copy,ctypes as ct,datetime as dt,hashlib,html,json,platform,subprocess,sys,threading,time,zlib
from pathlib import Path
import cv2
import numpy as np
from capture_mock.bindings import Config,Stats as SourceStats,native_config
from capture_mock.build import ROOT,monado_identity,sha
from experiment import MONADO_SOURCE_DIR, source_hashes as experiment_hashes, snapshot_sources, compare_normal_pixels
from pipeline_matrix.__main__ import View,Stats as PipelineStats,CALLBACK,fixture
from pipeline_matrix.observer import make_observer
from capture_mock.runner import validate_config as validate_capture_config
DIRECTORY=ROOT/"capture_e2e"
BUILD=ROOT/"build"/f"capture-e2e-{sys.platform}-{platform.machine()}"
LIBRARY=BUILD/("libwrl_capture_e2e.dylib" if sys.platform=="darwin" else "libwrl_capture_e2e.so")
SOURCE_CALLBACK=ct.CFUNCTYPE(ct.c_int,ct.c_void_p,ct.POINTER(View),ct.c_uint32)
class E2EStats(ct.Structure):
    _fields_=[("sources",SourceStats*4),("guard_seen",ct.c_uint64*4),
              ("guard_rejected",ct.c_uint64*4),("source_timed_out",ct.c_uint64),("finished",ct.c_uint64)]
def clock_ns():return time.clock_gettime_ns(time.CLOCK_MONOTONIC)
def save(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+"\n")
def build():
    identity=monado_identity();bin_dir=Path(sys.executable).parent
    commands=[
        [str(bin_dir/"cmake"),"-S",str(DIRECTORY/"native"),"-B",str(BUILD),"-G","Ninja",
         f"-DCMAKE_MAKE_PROGRAM={bin_dir/'ninja'}","-DCMAKE_BUILD_TYPE=RelWithDebInfo", f"-DMONADO_SOURCE_DIR={MONADO_SOURCE_DIR}"],
        [str(bin_dir/"cmake"),"--build",str(BUILD),"--parallel","2"],
        [str(bin_dir/"ctest"),"--test-dir",str(BUILD),"--output-on-failure"]]
    logs=[]
    for command in commands:
        p=subprocess.run(command,cwd=ROOT,text=True,capture_output=True,timeout=120)
        logs.append({"command":command,"exit":p.returncode,"output":p.stdout+p.stderr})
        if p.returncode:raise RuntimeError(logs[-1]["output"])
    save(BUILD/"build-record.json",{"role":"Native local component integration, not a full runtime","monado":identity,"commands":logs})
def library():
    lib=ct.CDLL(str(LIBRARY))
    lib.e2e_stats_size.restype=ct.c_uint32
    if lib.e2e_stats_size()!=ct.sizeof(E2EStats):raise RuntimeError("Native ABI mismatch; rebuild")
    lib.e2e_create.argtypes=[ct.POINTER(Config),ct.c_uint32,ct.c_uint32,ct.c_uint32,CALLBACK,SOURCE_CALLBACK,ct.c_void_p]
    lib.e2e_create.restype=ct.c_void_p
    lib.e2e_start.argtypes=[ct.c_void_p,ct.c_uint64];lib.e2e_start.restype=ct.c_int
    lib.e2e_stop.argtypes=[ct.c_void_p];lib.e2e_stop.restype=None
    lib.e2e_finish.argtypes=[ct.c_void_p,ct.c_uint32];lib.e2e_finish.restype=ct.c_int
    lib.e2e_stats.argtypes=[ct.c_void_p,ct.POINTER(E2EStats)];lib.e2e_stats.restype=None
    lib.e2e_pipeline_stats.argtypes=[ct.c_void_p,ct.POINTER(PipelineStats)];lib.e2e_pipeline_stats.restype=None
    lib.e2e_close.argtypes=[ct.c_void_p];lib.e2e_close.restype=None
    lib.e2e_format.argtypes=[ct.c_uint32];lib.e2e_format.restype=ct.c_uint32
    return lib
def load_case(name):
    catalog=json.loads((DIRECTORY/"cases.json").read_text())
    if name not in catalog["cases"]:raise ValueError("Unknown end-to-end scenario")
    return {**copy.deepcopy(catalog["defaults"]),**catalog["cases"][name]}
def validate(config):
    # Reuse established bounds and fault validation, without changing old config files.
    validate_capture_config({**config,"source_mode":"individual",
        "cv":{"max_corners":48,"quality_level":.05,"min_distance":8}})
    if config["converter"] not in ("none","RGB"):raise ValueError("Choose direct L8 or native RGB conversion")
    if config["pairing"] and (config["cameras"] not in (2,4) or config["consumer_delay_ms"] or
                              config["interrupt_camera"] is not None or config["malformed_kind"]):
        raise ValueError("Paired demo requires complete, unthrottled image streams")
class ExpectedPixels:
    def __init__(self,config):self.config=config
    def __getitem__(self,key):
        camera,sequence=key;cfg=self.config
        gray=fixture(cfg,camera,sequence)
        gray[0,9:12]=[cfg["seed"]&255,cfg["seed"]>>8,cfg["session_id"]]
        return gray
def authored_hashes():
    return experiment_hashes()
def percentile(values):
    if not values:return {"median_ms":None,"p95_ms":None,"max_ms":None}
    v=np.array(values,dtype=np.float64)/1e6
    return {"median_ms":float(np.median(v)),"p95_ms":float(np.percentile(v,95)),"max_ms":float(v.max())}
def run_case(name,config=None,output=None,do_build=True,shared_snapshot=None):
    cfg=copy.deepcopy(config if config is not None else load_case(name));validate(cfg)
    if do_build:build()
    lib=library();cv2.setNumThreads(1)
    stamp=dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output=Path(output or ROOT/"results/capture-e2e"/f"{name}-{stamp}").resolve()
    output.mkdir(parents=True,exist_ok=False)
    epoch=clock_ns()+50_000_000
    shared={**cfg,"input_format":"L8","virtual_fps_num":cfg["fps_num"],"virtual_fps_den":cfg["fps_den"],
            "first_pair_skew_ns":0}
    native={n:lib.e2e_format(i) for i,n in enumerate(("R8","L8","RGB"))}
    # Set the actual future epoch after source snapshots are safely prepared.
    source_records=[];source_errors=[];source_lock=threading.Lock()
    hashes=authored_hashes()
    manifest={"schema_version":1,"evidence_role":"Native mock Capture and measured local unchanged Monado components/CV",
              "scenario":name,"config":cfg,"seed":cfg["seed"],"created_at_utc":stamp,
              "fixture_config_sha256":hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest(),
              "source_hashes":hashes,"monado":monado_identity(),"python":sys.version,"platform":platform.platform(),
              "numpy":np.__version__,"opencv":cv2.__version__,"command":[sys.executable,*sys.argv],
              "clock":"CLOCK_MONOTONIC; synthetic capture/source time kept separate",
              "entry":"Native Capture factory via existing xrt_fs_slam_stream_start image-only interface",
              "modules":["native mock Capture L8 factory","new public-frame input guard",
                         "unchanged native converter (RGB route only)","native reference-lifetime observer",
                         "unchanged u_sink_queue.c","unchanged stereo pairer if explicitly configured",
                         "shared public-frame observer","OpenCV corners and annotated outputs"],
              "missing":["team production receiver/Builder","full tracking algorithms/runtime","Qualcomm/CSI/ISP","boards/physical camera sync"],
              "source_ready_timestamp":"Not exported; first public Capture-output observation is measured separately",
              "source_snapshot":"source_snapshot","status":"prepared",
              "native_library_hashes":{str(p.relative_to(BUILD)):sha(p) for p in BUILD.rglob("*") if p.suffix in (".so",".dylib")},
              "build":json.loads((BUILD/"build-record.json").read_text())}
    manifest["experiment_id"]=hashlib.sha256(json.dumps({"config":cfg,"source":hashes,"monado":manifest["monado"]["revision"]},sort_keys=True).encode()).hexdigest()
    manifest["run_id"]=f'{name}-{manifest["experiment_id"][:16]}-{stamp}'
    if shared_snapshot is None:
        snapshot_sources(output/"source_snapshot", hashes)
    else:
        snapshot_sources(shared_snapshot, hashes)
        (output/"source_snapshot").symlink_to(Path("..")/shared_snapshot.name, target_is_directory=True)
    save(output/"config.json",cfg);save(output/"manifest.json",manifest)
    epoch=clock_ns()+50_000_000
    observe,records,errors=make_observer(shared,output,native,ExpectedPixels(cfg),epoch)
    def source_observer(_,ptr,error):
        v=ptr.contents
        row={"camera_id":int(v.route),"sequence":int(v.sequence),"session_id":cfg["session_id"],
             "format":int(v.format),"width":int(v.width),"height":int(v.height),
             "stride":int(v.stride),"size":int(v.size),"source_id":int(v.source_id),
             "source_timestamp":int(v.source_timestamp),"published_timestamp":int(v.timestamp),
             "source_output_observed_ns":int(v.arrival_ns),"guard_error_code":int(error),
             "status":"rejected" if error else "forwarded"}
        if error:
            row["error"]={1:"Unsupported source format; expected native L8 grayscale",
                          2:"Invalid source row stride",3:"Invalid source payload size or null pixels",
                          4:"Unexpected source dimensions"}.get(int(error),"Input observer error")
        try:
            if not error:row["payload_crc32"]=zlib.crc32(ct.string_at(v.data,v.size))
            with source_lock:source_records.append(row)
            return 0
        except BaseException as error:
            source_errors.append(str(error));return 1
    cb=CALLBACK(observe);source_cb=SOURCE_CALLBACK(source_observer)
    configs=(Config*cfg["cameras"])(*(native_config(cfg,i) for i in range(cfg["cameras"])))
    handle=lib.e2e_create(configs,cfg["cameras"],2 if cfg["converter"]=="RGB" else 0,int(cfg["pairing"]),cb,source_cb,None)
    if not handle:raise RuntimeError("Native end-to-end session creation failed")
    src=E2EStats();pipeline=PipelineStats()
    try:
        # No producer starts before this manifest is on disk.
        manifest["scheduled_epoch_ns"]=epoch;manifest["status"]="running";save(output/"manifest.json",manifest)
        if lib.e2e_start(handle,epoch):raise RuntimeError("Native Capture start failed")
        if cfg["manual_stop_after_ms"] is not None:
            time.sleep(cfg["manual_stop_after_ms"]/1000);lib.e2e_stop(handle)
        finish=lib.e2e_finish(handle,cfg["timeout_ms"])
        lib.e2e_stats(handle,ct.byref(src));lib.e2e_pipeline_stats(handle,ct.byref(pipeline))
    finally:lib.e2e_close(handle)
    stages={key:int(getattr(pipeline,key)) for key,_ in pipeline._fields_}
    checks={"finish":finish==0,"sources_stopped":src.finished==1 and src.source_timed_out==0,
            "buffers_released":stages["live"]==0,"retained_pixels_after_source_destruction":stages["retained_valid"]==1,
            "public_observer_validated":not errors and not source_errors and stages["rejected"]==0,
            "receiver_counts":stages["received"]==stages["processed"]==len(records)}
    by_source={(r["camera_id"],r["sequence"]):r for r in source_records}
    streams={}
    for camera in range(cfg["cameras"]):
        native_source=src.sources[camera]
        produced=int(native_source.generated)
        inputs=[r for r in source_records if r["camera_id"]==camera]
        invalid=[r for r in inputs if r["status"]=="rejected"]
        frames=[r for r in records if r["camera_id"]==camera]
        sequences=[r["sequence"] for r in frames]
        accepted={r["sequence"] for r in inputs if r["status"]=="forwarded"}
        dropped=sorted(accepted-set(sequences))
        expected=cfg["interrupt_after"] if cfg["interrupt_camera"]==camera else cfg["frames"]
        rejected_expected=int(cfg["malformed_camera"]==camera and cfg["malformed_kind"]>0)
        camera_checks={
            "generated":0<produced<expected if cfg["manual_stop_after_ms"] is not None else produced==expected,
            "native_source_success":native_source.native_error==0 and native_source.producer_done==1,
            "source_accounted":produced==native_source.pushed==src.guard_seen[camera]==len(inputs),
            "guard_rejections":src.guard_rejected[camera]==len(invalid)==rejected_expected,
            "delivery_accounted":len(accepted)==len(frames)+len(dropped),
            "sequence_order":sequences==sorted(set(sequences)),
            "source_metadata_preserved":all(r["source_id"]==0x57524C47 and r["source_timestamp"]==by_source[(camera,r["sequence"])]["source_timestamp"] for r in frames),
            "clock_order":all(by_source[(camera,r["sequence"])]["source_output_observed_ns"]<=r["arrival_ns"]<=r.get("cv_done_ns",0) for r in frames),
            "drop_policy":bool(dropped) if cfg["consumer_delay_ms"] else not dropped}
        checks[f"camera_{camera}"]=all(camera_checks.values())
        times=[r["cv_done_ns"] for r in frames if "cv_done_ns" in r]
        fps=(len(times)-1)*1e9/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None
        streams[str(camera)]={"generated":produced,"guard_rejected":len(invalid),"received":len(frames),
            "processed":sum(r["status"]=="processed" for r in frames),"dropped":len(dropped),
            "missing_sequences":dropped,"duplicates":len(sequences)-len(set(sequences)),
            "processing_time":percentile([r["receive_to_cv_ns"] for r in frames if "receive_to_cv_ns" in r]),
            "source_output_to_receive":percentile([r["arrival_ns"]-by_source[(camera,r["sequence"])]["source_output_observed_ns"] for r in frames]),
            "observed_processed_fps":fps,"native_source_peak_frames":int(native_source.peak_live_frames),
            "source_live_frames_before_teardown":int(native_source.live_frames),"checks":camera_checks}
    checks["pipeline_input_accounted"]=stages["sent"]==stages["converted"]==sum(v["generated"]-v["guard_rejected"] for v in streams.values())
    if cfg["pairing"]:
        for pair in range(cfg["cameras"]//2):
            group=[r["camera_id"] for r in records if r["camera_id"]//2==pair]
            checks[f"pair_{pair}_ordered"]=group==[x for _ in range(cfg["frames"]) for x in (2*pair,2*pair+1)]
    if cfg["cameras"]>1 and cfg["manual_stop_after_ms"] is None:
        intervals=[(min(r["source_output_observed_ns"] for r in source_records if r["camera_id"]==c),
                    max(r["source_output_observed_ns"] for r in source_records if r["camera_id"]==c)) for c in range(cfg["cameras"])]
        checks["overlapping_source_intervals"]=min(e for _,e in intervals)>max(b for b,_ in intervals)
    (output/"source-frames.jsonl").write_text("".join(json.dumps(r)+"\n" for r in source_records))
    (output/"frames.jsonl").write_text("".join(json.dumps(r)+"\n" for r in records))
    summary={"schema_version":1,"scenario":name,"config":cfg,"local_prototype_passed":all(checks.values()),
             "streams":streams,"pipeline":stages,"checks":checks,"observer_errors":errors+source_errors,
             "input_errors":[{"camera":r["camera_id"],"sequence":r["sequence"],"code":r["guard_error_code"],"error":r["error"]} for r in source_records if r["guard_error_code"]],
             "output":str(output),"coverage":"Native Capture to real component graph to public CV diagnostic; not a production tracker or board",
             "timing":"Receive-to-CV includes FFI/GIL wait, independent image validation, conversion to grayscale and injected delay; excludes annotation saving"}
    summary["historical_pixel_baseline"]=compare_normal_pixels(summary)
    save(output/"summary.json",summary)
    rows=[]
    for c,s in streams.items():
        image=f"camera-{c}-annotated.png"
        rows.append(f'<tr><td>{c}</td><td>{s["generated"]}</td><td>{s["guard_rejected"]}</td><td>{s["processed"]}</td><td>{s["dropped"]}</td><td>{s["processing_time"]["median_ms"]}</td><td><img width="220" src="{image}" alt="camera {c}"/></td></tr>')
    (output/"index.html").write_text('<!doctype html><meta charset="utf-8"><title>Capture end-to-end</title><style>body{font:16px system-ui;margin:30px auto;max-width:1200px}td,th{padding:12px;border-bottom:1px solid #ddd}aside{padding:18px;background:#fff1cf}</style>'+
        f'<h1>{html.escape(name)}: {"PASS" if summary["local_prototype_passed"] else "FAIL"}</h1>'+
        '<aside>Native mock Capture → input guard → unchanged optional converter → unchanged queue → optional stereo pairing → public-frame CV observer. Local prototype; full tracker, Builder, Qualcomm and boards are untested.</aside>'+
        '<p>Time is measured on this host; source/exposure times are synthetic. Source-output observations are separate from downstream receipt.</p>'+
        '<table><tr><th>Camera</th><th>Generated</th><th>Guard rejects</th><th>CV processed</th><th>Queue drops</th><th>Median CV ms</th><th>Output</th></tr>'+"".join(rows)+
        '</table><p><a href="manifest.json">Manifest/source snapshot</a> · <a href="source-frames.jsonl">Capture output logs</a> · <a href="frames.jsonl">Receiver/CV logs</a> · <a href="summary.json">Checks</a></p>')
    manifest["status"]="complete";manifest["artifact_hashes"]={str(p.relative_to(output)):sha(p) for p in output.rglob("*") if p.is_file() and p!=output/"manifest.json"}
    save(output/"manifest.json",manifest);return summary
def suite(output=None,do_build=True):
    if do_build:build()
    stamp=dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output=Path(output or ROOT/"results/capture-e2e/suites"/stamp).resolve()
    output.mkdir(parents=True,exist_ok=False)
    catalog=json.loads((DIRECTORY/"cases.json").read_text())
    save(output/"suite-plan.json",{name:load_case(name) for name in catalog["cases"]})
    results=[]
    for name in catalog["cases"]:
        result=run_case(name,output=output/name,do_build=False,shared_snapshot=output/"_source_snapshot");results.append(result)
        print(json.dumps({"scenario":name,"passed":result["local_prototype_passed"]}),flush=True)
    save(output/"suite-summary.json",results)
    links="".join(f'<li><a href="{r["scenario"]}/index.html">{r["scenario"]}</a>: {"PASS" if r["local_prototype_passed"] else "FAIL"}</li>' for r in results)
    (output/"index.html").write_text('<!doctype html><meta charset="utf-8"><title>Native Capture end-to-end results</title><style>body{font:18px system-ui;max-width:1000px;margin:40px auto;padding:0 20px}aside{padding:20px;background:#fff1cf}li{padding:8px}</style><h1>Mock Capture → Monado components → CV</h1><aside>Local prototype with native source startup and completed L8 image buffers. Downstream processing components are unchanged. Team receiver/Builder, full tracking, Qualcomm, boards and physical sync remain untested.</aside><p>The normal cases use 640×480, target 30 FPS and 150 frames per camera. Failure cases test rejection, overload, interruption and cleanup. Source snapshots and independent pixel checks are supplied with every run.</p><ul>'+links+'</ul><p><a href="suite-plan.json">Declared cases</a> · <a href="suite-summary.json">Measurements</a></p>')
    print(json.dumps({"overview":str(output/"index.html"),"all_expected":all(r["local_prototype_passed"] for r in results)}),flush=True)
    return all(r["local_prototype_passed"] for r in results)
def main():
    parser=argparse.ArgumentParser();parser.add_argument("--scenario");parser.add_argument("--output",type=Path);parser.add_argument("--build-only",action="store_true")
    args=parser.parse_args()
    if args.build_only:build();return 0
    if args.scenario:
        result=run_case(args.scenario,output=args.output);print(json.dumps({"output":result["output"],"passed":result["local_prototype_passed"]}));return 0 if result["local_prototype_passed"] else 1
    return 0 if suite(args.output) else 1
if __name__=="__main__":raise SystemExit(main())

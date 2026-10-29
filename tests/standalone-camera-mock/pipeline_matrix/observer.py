"""Public-frame CV validation shared by component and native Capture runs."""
import ctypes as ct
import hashlib
import threading
import time
import cv2
import numpy as np

def clock_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)

def make_observer(config, output, native, expected_pixels, epoch):
    records=[];errors=[];lock=threading.Lock();samples=set()
    def receive(_,ptr):
        v=ptr.contents
        row={"camera_id":int(v.route),"sequence":int(v.sequence),"format":int(v.format),
             "width":int(v.width),"height":int(v.height),"stride":int(v.stride),"size":int(v.size),
             "source_id":int(v.source_id),"source_timestamp":int(v.source_timestamp),
             "published_timestamp":int(v.timestamp),"arrival_ns":int(v.arrival_ns)}
        try:
            if config["consumer_delay_ms"]:time.sleep(config["consumer_delay_ms"]/1000)
            channels=3 if v.format==native["RGB"] else 1
            assert v.format in native.values(),"Unknown native format"
            assert (v.width,v.height)==(config["width"],config["height"]),"Wrong dimensions"
            assert v.stride>=v.width*channels and v.size==v.stride*v.height,"Invalid layout"
            raw=ct.string_at(v.data,v.size)
            packed=np.frombuffer(raw,dtype=np.uint8).reshape(v.height,v.stride)[:,:v.width*channels]
            expected=expected_pixels[(v.route,v.sequence)]
            if channels==3:
                rgb=packed.reshape(v.height,v.width,3)
                assert np.array_equal(rgb,np.repeat(expected[:,:,None],3,axis=2)),"RGB conversion changed pixel identity"
                gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
            else:
                gray=packed.copy()
                assert np.array_equal(gray,expected),"Grayscale pixels changed"
            expected_format=config["converter"] if config["converter"]!="none" else config["input_format"]
            assert v.format==native[expected_format],"Unexpected output format"
            assert int(gray[0,4])==v.route,"Wrong camera route"
            assert int.from_bytes(gray[0,5:9].tobytes(),"little")==v.sequence,"Wrong sequence in pixels"
            synthetic=v.sequence*1_000_000_000*config["virtual_fps_den"]//config["virtual_fps_num"]
            skew=config["first_pair_skew_ns"] if v.sequence==0 and v.route%2 else 0
            assert v.source_timestamp==synthetic+skew,"Original source timestamp changed"
            expected_time=epoch+synthetic+(config["first_pair_skew_ns"]//2 if config["pairing"] and v.sequence==0 else skew)
            assert v.timestamp==expected_time,"Unexpected published timestamp"
            corners=cv2.goodFeaturesToTrack(gray,48,.05,8,blockSize=3)
            row["session_id"]=config.get("session_id")
            row["cv_done_ns"]=clock_ns();row["receive_to_cv_ns"]=row["cv_done_ns"]-v.arrival_ns
            row["corners"]=0 if corners is None else len(corners)
            assert row["corners"]>0,"No usable features"
            row["image_sha256"]=hashlib.sha256(gray.tobytes()).hexdigest();row["status"]="processed"
            if v.route not in samples:
                samples.add(v.route)
                annotated=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
                for xy in corners.reshape(-1,2):
                    cv2.circle(annotated,tuple(np.rint(xy).astype(int)),3,(0,210,40),1)
                assert cv2.imwrite(str(output/f"camera-{v.route}-annotated.png"),annotated)
                row["sample_file"]=f"camera-{v.route}-annotated.png"
            result=0
        except BaseException as error:
            row["status"]="rejected";row["error"]=str(error);errors.append(str(error));result=1
        with lock:records.append(row)
        return result
    return receive,records,errors

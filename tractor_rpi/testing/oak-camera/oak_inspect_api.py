#!/usr/bin/env python3

import depthai as dai

def inspect_api():
    """Inspect the actual DepthAI API available"""
    
    print(f"DepthAI version: {dai.__version__}")
    print("\n" + "="*50)
    
    # Inspect Pipeline methods
    pipeline = dai.Pipeline()
    pipeline_methods = [method for method in dir(pipeline) if not method.startswith('_')]
    
    print("Available Pipeline methods:")
    for method in sorted(pipeline_methods):
        if 'create' in method.lower() or 'camera' in method.lower():
            print(f"  - {method}")
    
    print("\nAll create methods:")
    create_methods = [method for method in pipeline_methods if method.startswith('create')]
    for method in sorted(create_methods):
        print(f"  - {method}")
    
    # Check dai.node contents
    print(f"\nAvailable dai.node classes:")
    if hasattr(dai, 'node'):
        node_classes = [attr for attr in dir(dai.node) if not attr.startswith('_')]
        for cls in sorted(node_classes):
            if 'camera' in cls.lower():
                print(f"  - dai.node.{cls} ⭐")
            else:
                print(f"  - dai.node.{cls}")
    
    # Check for XLinkOut
    print(f"\nXLinkOut methods:")
    xlink_methods = [method for method in pipeline_methods if 'xlink' in method.lower() or 'link' in method.lower()]
    for method in sorted(xlink_methods):
        print(f"  - {method}")
    
    # Test different camera creation approaches
    print(f"\n" + "="*50)
    print("Testing camera creation approaches:")
    
    approaches = [
        ("pipeline.createColorCamera()", lambda: pipeline.createColorCamera()),
        ("pipeline.create(dai.node.ColorCamera)", lambda: pipeline.create(dai.node.ColorCamera)),
        ("pipeline.create(dai.node.Camera)", lambda: pipeline.create(dai.node.Camera)),
        ("pipeline.createCamera()", lambda: pipeline.createCamera() if hasattr(pipeline, 'createCamera') else None),
    ]
    
    working_approach = None
    
    for name, func in approaches:
        try:
            result = func()
            if result is not None:
                print(f"  ✅ {name} - SUCCESS")
                if working_approach is None:
                    working_approach = (name, func)
                    
                # Check available methods on the camera object
                cam_methods = [method for method in dir(result) if not method.startswith('_')]
                preview_methods = [method for method in cam_methods if 'preview' in method.lower()]
                size_methods = [method for method in cam_methods if 'size' in method.lower()]
                
                print(f"    Preview methods: {preview_methods}")
                print(f"    Size methods: {size_methods}")
                
        except Exception as e:
            print(f"  ❌ {name} - FAILED: {e}")
    
    # Test XLinkOut creation
    print(f"\nTesting XLinkOut creation:")
    xlink_approaches = [
        ("pipeline.createXLinkOut()", lambda: pipeline.createXLinkOut()),
        ("pipeline.create(dai.node.XLinkOut)", lambda: pipeline.create(dai.node.XLinkOut)),
    ]
    
    working_xlink = None
    
    for name, func in xlink_approaches:
        try:
            result = func()
            if result is not None:
                print(f"  ✅ {name} - SUCCESS")
                if working_xlink is None:
                    working_xlink = (name, func)
        except Exception as e:
            print(f"  ❌ {name} - FAILED: {e}")
    
    # Generate working code template
    if working_approach and working_xlink:
        print(f"\n" + "="*50)
        print("WORKING CODE TEMPLATE:")
        print(f"")
        print(f"pipeline = dai.Pipeline()")
        print(f"cam = {working_approach[0].split('(')[0]}()")
        print(f"# cam.setPreviewSize(640, 480)  # Test if this works")
        print(f"out = {working_xlink[0].split('(')[0]}()")
        print(f"out.setStreamName('video')")
        print(f"# cam.preview.link(out.input)  # Test if this works")
    
    return working_approach, working_xlink

if __name__ == "__main__":
    inspect_api()

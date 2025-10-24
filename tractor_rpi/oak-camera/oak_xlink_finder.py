#!/usr/bin/env python3

import depthai as dai

def find_xlink():
    """Find the correct way to create XLinkOut"""
    
    print("Searching for XLinkOut...")
    
    # Check main dai module
    dai_attrs = [attr for attr in dir(dai) if not attr.startswith('_')]
    print("\nChecking main dai module for 'Link' or 'Out':")
    for attr in sorted(dai_attrs):
        if 'link' in attr.lower() or 'out' in attr.lower():
            print(f"  - dai.{attr}")
    
    # Check pipeline methods more thoroughly
    pipeline = dai.Pipeline()
    pipeline_methods = [method for method in dir(pipeline) if not method.startswith('_')]
    
    print("\nAll pipeline methods containing 'Link' or 'Out':")
    for method in sorted(pipeline_methods):
        if 'link' in method.lower() or 'out' in method.lower():
            print(f"  - pipeline.{method}")
    
    print("\nAll pipeline methods (looking for output-related):")
    for method in sorted(pipeline_methods):
        if any(word in method.lower() for word in ['output', 'stream', 'queue', 'send']):
            print(f"  - pipeline.{method}")
    
    # Test creating XLinkOut different ways
    print(f"\nTesting XLinkOut creation approaches:")
    
    approaches = [
        ("dai.XLinkOut()", lambda: dai.XLinkOut() if hasattr(dai, 'XLinkOut') else None),
        ("pipeline.createXLinkOut()", lambda: pipeline.createXLinkOut() if hasattr(pipeline, 'createXLinkOut') else None),
        ("pipeline.create('XLinkOut')", lambda: pipeline.create('XLinkOut')),
    ]
    
    # Also try looking in dai.node more carefully
    if hasattr(dai, 'node'):
        node_classes = [attr for attr in dir(dai.node) if 'link' in attr.lower() or 'out' in attr.lower()]
        print(f"\nFound in dai.node: {node_classes}")
        
        for cls_name in node_classes:
            try:
                approaches.append((f"pipeline.create(dai.node.{cls_name})", 
                                lambda cn=cls_name: pipeline.create(getattr(dai.node, cn))))
            except:
                pass
    
    # Test each approach
    working_xlink = None
    for name, func in approaches:
        try:
            result = func()
            if result is not None:
                print(f"  ✅ {name} - SUCCESS")
                if working_xlink is None:
                    working_xlink = (name, result)
                    # Check methods
                    methods = [m for m in dir(result) if not m.startswith('_') and 'stream' in m.lower()]
                    print(f"    Stream methods: {methods}")
        except Exception as e:
            print(f"  ❌ {name} - FAILED: {e}")
    
    # Try a complete minimal pipeline
    if working_xlink is None:
        print(f"\nTrying alternative approach - check all dai classes:")
        for attr in sorted(dir(dai)):
            if not attr.startswith('_') and ('Link' in attr or 'Out' in attr):
                try:
                    cls = getattr(dai, attr)
                    if hasattr(cls, '__call__'):  # It's a class/function
                        print(f"  Found: dai.{attr}")
                        obj = cls()
                        print(f"    Created successfully: {type(obj)}")
                        if hasattr(obj, 'setStreamName'):
                            working_xlink = (f"dai.{attr}()", obj)
                            break
                except Exception as e:
                    print(f"  dai.{attr} failed: {e}")
    
    return working_xlink

if __name__ == "__main__":
    result = find_xlink()
    if result:
        print(f"\n🎉 Found working XLinkOut: {result[0]}")
    else:
        print(f"\n😞 No working XLinkOut found")

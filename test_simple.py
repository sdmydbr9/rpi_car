#!/usr/bin/env python3
"""
Simple test to verify Smart Driver integration
"""

import ast
import sys

def test_main_structure():
    """Test that main.py has the required autonomous driving components"""
    print("🧪 Testing Smart Driver Integration")
    print("=" * 50)
    
    try:
        with open('main.py', 'r') as f:
            content = f.read()
        
        # Parse the AST
        tree = ast.parse(content)
        
        # Check for State enum
        state_found = False
        autonomous_vars_found = []
        drive_autonomous_found = False
        
        for node in ast.walk(tree):
            # Check for State enum
            if isinstance(node, ast.ClassDef) and node.name == 'State':
                state_found = True
                print("✅ State enum found")
            
            # Check for drive_autonomous function
            if isinstance(node, ast.FunctionDef) and node.name == 'drive_autonomous':
                drive_autonomous_found = True
                print("✅ drive_autonomous() function found")
            
            # Check for autonomous variables in car_state
            if isinstance(node, ast.Dict):
                for key in node.keys:
                    if isinstance(key, ast.Constant) and isinstance(key.value, str):
                        if key.value.startswith('autonomous'):
                            autonomous_vars_found.append(key.value)
        
        print(f"\n✅ Autonomous variables found: {len(autonomous_vars_found)}")
        for var in autonomous_vars_found:
            print(f"  - {var}")
        
        # Check for HTTP endpoints
        endpoints = []
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and hasattr(node, 'decorator_list'):
                for decorator in node.decorator_list:
                    if isinstance(decorator, ast.Name) and decorator.id == 'app':
                        if hasattr(decorator, 'attr') and decorator.attr == 'route':
                            endpoints.append(node.name)
        
        autonomous_endpoints = [ep for ep in endpoints if 'autonomous' in ep]
        print(f"\n✅ Autonomous endpoints found: {len(autonomous_endpoints)}")
        for ep in autonomous_endpoints:
            print(f"  - {ep}")
        
        # Summary
        all_checks = [
            state_found,
            drive_autonomous_found,
            len(autonomous_vars_found) >= 5,
            len(autonomous_endpoints) >= 3
        ]
        
        if all(all_checks):
            print("\n🎉 All Smart Driver components successfully integrated!")
            return True
        else:
            print("\n❌ Some components missing")
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    success = test_main_structure()
    sys.exit(0 if success else 1)

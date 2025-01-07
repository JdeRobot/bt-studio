import os
import argparse
import shutil
import zipfile
import re
import subprocess

##############################################################################
# Helper functions
##############################################################################

# Collect unique imports from files
def get_unique_imports(actions):

    unique_imports = set()

    for action in actions:
        lines = action["content"].splitlines()

        for line in lines:
            # Using regex to find lines that don't start with '#' and have 'import ...' or 'from ... import ...'
            match = re.search(r"^(?!#.*)(?:import|from)\s+(\w+)", line)
            if match:
                unique_imports.add(match.group(1))

    return list(unique_imports)

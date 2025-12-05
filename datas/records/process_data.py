import re

# Input and output file paths
input_file = r"C:\Users\thomm\.vscode\Folders\Github\Mobile-Robot\datas\records\Tom_meilleur.txt" #To replace with actual path
output_file = r"C:\Users\thomm\.vscode\Folders\Github\Mobile-Robot\datas\records\best.csv" #To replace with actual path

# Read the input file
with open(input_file, 'r', encoding='utf-8-sig') as f:
    lines = f.readlines()

# Process the data
output_lines = []
index = 1

# Parse header to find column indices
header = lines[0].strip()
# Remove BOM and hidden Unicode characters
header = header.encode('utf-8-sig').decode('utf-8')
header = ''.join(c for c in header if ord(c) >= 32 or c in '\t\n')
# Handle both comma-separated and space-separated formats
if ',' in header:
    header_cols = header.split(',')
else:
    header_cols = header.split()
header_cols = [col.strip() for col in header_cols]  # Strip whitespace from each column

try:
    x_est_idx = header_cols.index("\ufeffx_est")
    x_ref_idx = header_cols.index('x_ref')
except ValueError as e:
    print(f"Error: {e}")
    print(f"Available columns: {header_cols}")
    exit(1)

# Add the output header
output_lines.append("index,x_est,x_ref")

# Process data lines (skip header)
data_lines = lines[1:]

for i, line in enumerate(data_lines):
    line = line.strip()
    # Remove hidden Unicode characters
    line = ''.join(c for c in line if ord(c) >= 32 or c in '\t\n')
    if not line:  # Skip empty lines
        continue
    
    # Handle both comma-separated and space-separated formats
    if ',' in line:
        cols = line.split(',')
    else:
        cols = line.split()
    cols = [col.strip() for col in cols]  # Strip whitespace from each value
    x_est = cols[x_est_idx]
    x_ref = cols[x_ref_idx]
    output_lines.append(f"{index},{x_est},{x_ref}")
    index += 1

# Write to output file
with open(output_file, 'w', encoding='utf-8') as f:
    f.write('\n'.join(output_lines))

print(f"Processing complete!")
print(f"Input file: {input_file}")
print(f"Output file: {output_file}")
print(f"Total rows processed: {index - 1}")

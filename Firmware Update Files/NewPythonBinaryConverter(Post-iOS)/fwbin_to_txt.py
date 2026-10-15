import argparse
from pathlib import Path

def convert_bin_to_compact_txt(bin_path: str, output_path: str, var_name: str = "firmware_bin", chunk_size: int = 1080):
    input_file = Path(bin_path)
    base_output = Path(output_path)
    
    if not input_file.exists():
        raise FileNotFoundError(f"Input file not found: {bin_path}")
        
    data = input_file.read_bytes()
    
    # Break binary data into 1,080-byte chunks
    chunks = [data[i:i + chunk_size] for i in range(0, len(data), chunk_size)]
    
    # Split chunks equally between P1 and P2 (P1 gets the extra chunk if total is odd)
    mid = (len(chunks) + 1) // 2
    chunks_p1 = chunks[:mid]
    chunks_p2 = chunks[mid:]
    
    # Calculate exact byte count for each file
    bytes_p1 = sum(len(c) for c in chunks_p1)
    bytes_p2 = sum(len(c) for c in chunks_p2)
    
    # Format bytes within chunks
    formatted_p1 = [",".join(f"{b:02X}" for b in c) for c in chunks_p1]
    formatted_p2 = [",".join(f"{b:02X}" for b in c) for c in chunks_p2]
    
    # Build single-line strings for each part
    content_p1 = f"{var_name}P1|{bytes_p1}|" + "|".join(formatted_p1) if formatted_p1 else f"{var_name}P1|0|"
    content_p2 = f"{var_name}P2|{bytes_p2}|" + "|".join(formatted_p2) if formatted_p2 else f"{var_name}P2|0|"
    
    # Derive output paths ending in P1.txt and P2.txt
    stem = base_output.stem
    ext = base_output.suffix if base_output.suffix else ".txt"
    path_p1 = base_output.with_name(f"{stem}P1{ext}")
    path_p2 = base_output.with_name(f"{stem}P2{ext}")
    
    # Write files without trailing newlines
    with open(path_p1, "w", newline="") as f:
        f.write(content_p1)
    with open(path_p2, "w", newline="") as f:
        f.write(content_p2)
        
    print(f"Successfully generated:\n - {path_p1} ({bytes_p1} bytes, {len(chunks_p1)} chunks)\n - {path_p2} ({bytes_p2} bytes, {len(chunks_p2)} chunks)")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert .bin file to split single-line text files.")
    parser.add_argument("input", help="Path to input .bin file")
    parser.add_argument("output", help="Path to output text base name (e.g., firmware.txt)")
    parser.add_argument("--var", default="firmware_bin", help="Array name prefix (default: firmware_bin)")
    
    args = parser.parse_args()
    convert_bin_to_compact_txt(args.input, args.output, args.var)
import os

def generate_random_hex_key(length=32):
    # Generate random bytes
    random_bytes = os.urandom(length)
    # Format each byte as '0xNN'
    hex_key = ', '.join(f'0x{byte:02x}' for byte in random_bytes)
    return hex_key

if __name__ == "__main__":
    hex_key = generate_random_hex_key()
    print(f"Random 32-byte hex key: {hex_key}")

    hex_key = generate_random_hex_key(16)
    print(f"Random 16-byte hex key: {hex_key}")
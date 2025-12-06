# 🎯 BLE Token Strategy - Complete Implementation Guide for Jules

**Status**: Ready for Jules AI Implementation  
**Priority**: CRITICAL - Blocks camera login  
**Complexity**: HIGH (Protocol + BLE + Token Extraction)  
**Estimated Effort**: 3-4 hours

---

## 📋 EXECUTIVE SUMMARY

### The Problem
Your Raspberry Pi fails to authenticate with the camera because:

1. **Hardcoded Token is Stale**: `"MzlB36X/IVo8ZzI5rG9j1w=="` (25 chars)
   - Changes on **every BLE wake cycle**
   - Real token is 45 chars: `"I3mbwVIxJQgnSB9GJKNk5Bvv/y+g8+MX/HVCMnCqyUo="`

2. **Missing Token Extraction**: No code extracts token from BLE notification
   - The Android app gets token from BLE after wake
   - You ignore this notification completely

3. **Wrong Protocol Understanding**: You thought it was "JSON over UDP"
   - **REALITY**: ARTEMIS Binary Protocol wraps everything
   - Magic: `0xf1d0` + "ARTEMIS" string + Binary header + Payload

### The Solution (3 Components)

```
┌─────────────────────────────────────────────┐
│  1. BLE Wake                               │
│     └─ Send wake packet via BLE            │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│  2. Token Extraction (NEW!)                 │
│     └─ Listen for BLE notification         │
│     └─ Extract Base64 token from payload   │
│     └─ Extract sequence bytes (0x2b, etc)  │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│  3. ARTEMIS UDP Login                       │
│     └─ Build: f1d0 + ARTEMIS + Token       │
│     └─ Use extracted token (not hardcoded) │
│     └─ Use extracted sequence (dynamic)    │
└─────────────────────────────────────────────┘
```

---

## 🔬 TECHNICAL ANALYSIS

### Part 1: The ARTEMIS Binary Protocol

**What is ARTEMIS?**
- Binary protocol used by camera for ALL communication
- Wraps JSON commands, Base64 tokens, binary data
- UDP-based with sequence numbers

**Packet Structure**:

```
┌─────────────────────────────────────────────────────┐
│  0-1:   Magic Number (0xf1d0)                       │
│         └─ Identifies as ARTEMIS packet             │
├─────────────────────────────────────────────────────┤
│  2-3:   Payload Length                              │
│         └─ Size of data after header                │
├─────────────────────────────────────────────────────┤
│  4-5:   Header Field 1 (d1 00)                      │
│         └─ Purpose unclear, constant               │
├─────────────────────────────────────────────────────┤
│  6-7:   Sequence Counter                            │
│         └─ Increments: 00 05, 00 06, etc           │
├─────────────────────────────────────────────────────┤
│  8-17:  Protocol Identifier ("ARTEMIS\x00")         │
│         └─ 7-byte string + null terminator          │
├─────────────────────────────────────────────────────┤
│  18-25: Version Field (02 00 00 00)                 │
│         └─ Always observed as 02 00 00 00           │
├─────────────────────────────────────────────────────┤
│  26-29: Dynamic Sequence from BLE (0x2b 00 00 00)   │
│         └─ Changes after BLE handshake             │
│         └─ First login: 0x2b (43)                   │
│         └─ Retry:      0x2c (44)                    │
│         └─ Next:       0x2e (46)                    │
├─────────────────────────────────────────────────────┤
│  30-31: Token Length (0x2d 00 00 00 = 45 bytes)    │
│         └─ Base64 token length                      │
├─────────────────────────────────────────────────────┤
│  32+:   Payload (Base64 token OR JSON)              │
│         └─ Login: Base64 token (45 chars)           │
│         └─ Commands: JSON string                    │
│         └─ Null terminator at end                   │
└─────────────────────────────────────────────────────┘
```

**Real Example from TCP Dump**:

```hex
f1d0 0045          ← Magic + Length (69 bytes)
d100 0005          ← Header fields
4152 5445 4d49 5300 ← "ARTEMIS\x00"
0200 0000          ← Version
2b00 0000          ← Sequence 0x2b (from BLE!)
2d00 0000          ← Token length (45)
4933 6d62 7756 4978 4a51 676e 5342 3947... ← Base64 token
```

**Decoded**:
- Magic: 0xf1d0 ✓
- Length: 69 bytes ✓
- Protocol: "ARTEMIS" ✓
- Sequence: 0x2b = 43 decimal (increments on retry)
- Token: "I3mbwVIxJQgnSB9GJKNk5Bvv/y+g8+MX/HVCMnCqyUo=" (45 chars)

### Part 2: BLE Token Extraction

**Where does the token come from?**

After BLE wake, the camera sends a notification containing:
- New Auth Token (Base64, 45 characters)
- Sequence Byte (0x2b, 0x2c, etc. - increments on retry)
- Possibly other metadata

**BLE Notification Structure** (Educated Guess from Smartphone Behavior):

```
Characteristic: Some notification UUID
Payload: [Token Length][Sequence][Token Data]
Example: 2d 00 00 00 2b 00 00 00 49 33 6d 62 77 56 ...
         ├─ Token Len ─┤ ├─ Seq ─┤ ├─ Token Starts Here ─►
```

**What We Know For Certain**:
- Token is 45 bytes of Base64
- Sequence byte increments (0x2b → 0x2c → 0x2d → 0x2e)
- NOT using hardcoded values
- Android app extracts these correctly → camera responds with LOGIN OK

### Part 3: The Hardcoded vs. Dynamic Problem

| Aspect | Current (❌ Fails) | Required (✅ Works) |
|--------|-------------------|-------------------|
| Token Source | Hardcoded in code | Extracted from BLE |
| Token Value | `MzlB36X/IVo8ZzI5rG9j1w==` | `I3mbwVIxJQgnSB9GJKNk5Bvv/y+g8+MX/HVCMnCqyUo=` |
| Token Length | 25 characters | 45 characters |
| Sequence | `02 00 01 00` static | `2b 00 00 00` from BLE, increments |
| Token Lifetime | Permanent (wrong!) | Per-session (correct) |
| Result | TIMEOUT / AUTH FAILED | LOGIN OK ✅ |

---

## 🛠️ IMPLEMENTATION ROADMAP FOR JULES

### Phase 1: BLE Token Listener (1-2 hours)

**Task 1.1: Add Token Listener Class**

```python
# File: src/ble_token_listener.py

from bleak import BleakClient
from bleak.exc import BleakError
import asyncio
import logging

class TokenListener:
    """Extracts auth token from BLE notification after wake."""
    
    def __init__(self, device_mac: str, logger=None):
        self.device_mac = device_mac
        self.logger = logger or logging.getLogger(__name__)
        self.token = None
        self.sequence = None
        self.token_event = asyncio.Event()
    
    async def listen(self, timeout=10):
        """
        Connect to camera via BLE and listen for token notification.
        
        Returns:
            dict: {"token": "I3mbwVIx...", "sequence": 0x2b}
        """
        try:
            async with BleakClient(self.device_mac) as client:
                # TODO: Discover characteristic UUIDs from camera
                # This is the tricky part - need to sniff which char sends token
                
                # Register notification handler
                await client.start_notify(UNKNOWN_CHAR_UUID, self._notification_handler)
                
                # Wait for notification with timeout
                await asyncio.wait_for(self.token_event.wait(), timeout)
                
                return {
                    "token": self.token,
                    "sequence": self.sequence
                }
        except asyncio.TimeoutError:
            self.logger.error("Token extraction timeout")
            raise
        except BleakError as e:
            self.logger.error(f"BLE error: {e}")
            raise
    
    def _notification_handler(self, sender, data):
        """Parse token from BLE notification."""
        # Expected structure (from analysis):
        # [2d 00 00 00] [2b 00 00 00] [I3mbwVIx...Token...]
        # Token length    Sequence      Base64 token (45 bytes)
        
        try:
            # Parse 4-byte length (little-endian)
            token_len = int.from_bytes(data[0:4], 'little')
            self.logger.debug(f"Token length: {token_len}")
            
            # Parse 4-byte sequence
            self.sequence = data[4:8]
            self.logger.debug(f"Sequence bytes: {self.sequence.hex()}")
            
            # Extract token
            token_bytes = data[8:8+token_len]
            self.token = token_bytes.decode('ascii').strip('\x00')
            self.logger.info(f"Token extracted: {self.token}")
            
            # Signal that token is ready
            self.token_event.set()
        except Exception as e:
            self.logger.error(f"Token parsing error: {e}")

```

**Task 1.2: Discover Token Characteristic UUID**

This is CRITICAL - you need to know which BLE characteristic sends the token!

```bash
# Option A: Use existing BLE scan tool
sudo bluetoothctl
> scan on
> [wait for camera to appear]
> info C6:1E:0D:XX:XX:XX  [camera MAC]
> attributes

# Option B: Use custom BLE scanner
python3 ble_characteristic_scanner.py <camera_mac>
# Output: Lists all characteristics and their UUIDs
```

**Expected Output**:
```
Service: Unknown (generic_access)
  Characteristic 1: 2a00 (Device Name)
  Characteristic 2: 2a01 (Appearance)
  ...
Service: Unknown (vendor-specific?)
  Characteristic N: ???? (SENDS TOKEN - THIS ONE!)
  Characteristic N+1: ???? (Receives commands?)
```

**Action for Jules**:
- Add BLE characteristic scanner tool
- Document which UUID sends token notification
- Add HARDCODED comment with discovered UUID

---

### Phase 2: Token-Aware Camera Client (1-1.5 hours)

**Task 2.1: Modify camera_client.py**

Replace hardcoded token with injected token:

```python
# File: src/camera_client.py

class CameraClient:
    """Reworked camera client that uses dynamic BLE token."""
    
    def __init__(self, camera_ip: str, logger=None):
        self.camera_ip = camera_ip
        self.logger = logger or logging.getLogger(__name__)
        self.session_token = None  # ← NEW!
        self.sequence_bytes = None # ← NEW!
    
    def set_session_credentials(self, token: str, sequence: bytes):
        """
        Set credentials extracted from BLE after wake.
        
        Args:
            token: Base64 auth token (45 chars) from BLE notification
            sequence: 4 bytes from BLE (e.g., b'\x2b\x00\x00\x00')
        """
        if len(token) != 45:
            self.logger.warning(f"Token length {len(token)} != 45, may fail")
        
        self.session_token = token
        self.sequence_bytes = sequence
        self.logger.info(f"Session credentials set. Token: {token[:20]}..., Seq: {sequence.hex()}")
    
    def _build_login_payload(self) -> bytes:
        """
        Build ARTEMIS binary login packet with DYNAMIC token.
        
        Structure:
          0-1:   0xf1d0 (magic)
          2-3:   payload length
          4-5:   0xd100 (constant)
          6-7:   0x0005 (constant)
          8-17:  "ARTEMIS\x00" (7 bytes + null)
          18-25: 0x02000000 (version)
          26-29: sequence bytes from BLE (e.g., 0x2b000000)
          30-33: token length as 4-byte int (0x2d000000 = 45)
          34+:   token as ASCII + null terminator
        """
        if not self.session_token or not self.sequence_bytes:
            raise ValueError("Session credentials not set. Call set_session_credentials() first!")
        
        token_bytes = self.session_token.encode('ascii')
        
        # Build payload components
        magic = b'\xf1\xd0'
        proto_string = b'ARTEMIS\x00'
        version = b'\x02\x00\x00\x00'
        token_len = (len(token_bytes)).to_bytes(4, 'little')
        
        # Assemble payload
        payload = (
            version +
            self.sequence_bytes +  # Dynamic sequence from BLE!
            token_len +
            token_bytes +
            b'\x00'  # Null terminator
        )
        
        # Calculate total length (after magic)
        total_len = 2 + len(proto_string) + len(payload)  # 2 for magic itself
        length_bytes = total_len.to_bytes(2, 'big')
        
        # Full packet
        packet = (
            magic +
            length_bytes +
            b'\xd1\x00' +  # Header constant
            b'\x00\x05' +  # Sequence constant
            proto_string +
            payload
        )
        
        self.logger.debug(f"Login payload: {packet.hex()}")
        return packet
    
    def login(self) -> bool:
        """
        Authenticate with camera using BLE-extracted token.
        
        Returns:
            True if login succeeds
        """
        if not self.session_token:
            self.logger.error("No session token set! Call set_session_credentials() first")
            return False
        
        try:
            payload = self._build_login_payload()
            
            # Send UDP packet
            response = self._send_udp(self.camera_ip, 10000, payload, timeout=5)
            
            if response:
                self.logger.info("✅ LOGIN SUCCESSFUL!")
                return True
            else:
                self.logger.error("❌ LOGIN FAILED - No response from camera")
                return False
        
        except Exception as e:
            self.logger.error(f"❌ LOGIN ERROR: {e}")
            return False
    
    def _send_udp(self, ip: str, port: int, data: bytes, timeout=5) -> bytes:
        """Send UDP packet and get response."""
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        try:
            sock.sendto(data, (ip, port))
            response, _ = sock.recvfrom(1024)
            return response
        except socket.timeout:
            return None
        finally:
            sock.close()

```

**Task 2.2: Add Test Variants**

Keep these for testing different scenarios:

```python
# File: src/test_camera_login.py

class TestVariants:
    """Test different token scenarios."""
    
    def __init__(self, client: CameraClient):
        self.client = client
    
    def test_with_extracted_token(self, token: str, sequence: bytes):
        """Test with real BLE-extracted token."""
        print(f"Test 1: Using extracted token {token[:20]}...")
        self.client.set_session_credentials(token, sequence)
        return self.client.login()
    
    def test_with_hardcoded_fallback(self):
        """Test with old hardcoded token (should fail)."""
        print("Test 2: Using hardcoded token (expect failure)...")
        self.client.set_session_credentials(
            "MzlB36X/IVo8ZzI5rG9j1w==",
            b'\x02\x00\x01\x00'
        )
        return self.client.login()
    
    def test_sequence_increment(self, base_token: str, base_sequence: bytes):
        """Test with incremented sequence (retry scenario)."""
        print("Test 3: Testing sequence increment on retry...")
        
        # First attempt
        seq1 = base_sequence
        self.client.set_session_credentials(base_token, seq1)
        result1 = self.client.login()
        
        # Second attempt (sequence increments)
        seq2 = bytes([base_sequence[0] + 1]) + base_sequence[1:]
        self.client.set_session_credentials(base_token, seq2)
        result2 = self.client.login()
        
        return (result1, result2)

```

---

### Phase 3: Main Orchestration (1 hour)

**Task 3.1: Update main.py with Full Workflow**

```python
# File: src/main.py (updated)

import asyncio
import logging
from ble_manager import BLEManager
from ble_token_listener import TokenListener
from camera_client import CameraClient

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

async def main():
    """
    Complete workflow: BLE Wake → Token Extraction → UDP Login
    """
    
    CAMERA_MAC = "C6:1E:0D:XX:XX:XX"  # Replace with actual
    CAMERA_IP = "192.168.43.1"
    
    try:
        # PHASE 1: BLE Wake
        logger.info("=" * 60)
        logger.info("PHASE 1: BLE WAKE")
        logger.info("=" * 60)
        
        ble_mgr = BLEManager()
        ble_mgr.wake_camera(CAMERA_MAC)
        
        # Wait for camera to stabilize
        await asyncio.sleep(2)
        
        # PHASE 2: Token Extraction
        logger.info("=" * 60)
        logger.info("PHASE 2: TOKEN EXTRACTION")
        logger.info("=" * 60)
        
        token_listener = TokenListener(CAMERA_MAC, logger)
        creds = await token_listener.listen(timeout=10)
        
        logger.info(f"✅ Token extracted: {creds['token'][:20]}...")
        logger.info(f"✅ Sequence: {creds['sequence'].hex()}")
        
        # PHASE 3: UDP Login with Extracted Token
        logger.info("=" * 60)
        logger.info("PHASE 3: UDP LOGIN")
        logger.info("=" * 60)
        
        camera = CameraClient(CAMERA_IP, logger)
        camera.set_session_credentials(creds['token'], creds['sequence'])
        
        success = camera.login()
        
        if success:
            logger.info("🎉 COMPLETE SUCCESS! Camera authenticated!")
            return True
        else:
            logger.error("❌ Login failed despite correct token")
            return False
    
    except asyncio.TimeoutError:
        logger.error("❌ Token extraction timeout - camera didn't send notification")
        return False
    except Exception as e:
        logger.error(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    result = asyncio.run(main())
    exit(0 if result else 1)

```

**Task 3.2: Add Error Handling & Retries**

```python
# File: src/main_with_retries.py

async def main_with_retries(max_attempts=3):
    """Robust version with retry logic."""
    
    for attempt in range(1, max_attempts + 1):
        logger.info(f"\n{'='*60}")
        logger.info(f"ATTEMPT {attempt}/{max_attempts}")
        logger.info(f"{'='*60}\n")
        
        try:
            # BLE Wake
            ble_mgr = BLEManager()
            ble_mgr.wake_camera(CAMERA_MAC)
            await asyncio.sleep(2)
            
            # Token extraction
            token_listener = TokenListener(CAMERA_MAC, logger)
            creds = await token_listener.listen(timeout=10)
            
            # Increment sequence for retries
            if attempt > 1:
                seq_val = int.from_bytes(creds['sequence'][:1], 'little')
                creds['sequence'] = bytes([seq_val + attempt - 1]) + creds['sequence'][1:]
            
            # Login
            camera = CameraClient(CAMERA_IP, logger)
            camera.set_session_credentials(creds['token'], creds['sequence'])
            
            if camera.login():
                logger.info(f"✅ SUCCESS on attempt {attempt}!")
                return True
            
            logger.warning(f"❌ Attempt {attempt} failed, retrying...")
            await asyncio.sleep(2)
        
        except Exception as e:
            logger.error(f"❌ Attempt {attempt} error: {e}")
            if attempt < max_attempts:
                await asyncio.sleep(2)
            continue
    
    logger.error("❌ ALL ATTEMPTS FAILED")
    return False

```

---

## 🧪 TESTING & VALIDATION

### Test 1: BLE Discovery
```bash
# Verify which characteristic sends the token
python3 src/ble_characteristic_scanner.py C6:1E:0D:XX:XX:XX

# Expected output:
# Service: uuid-XXXXX (vendor-specific)
#   Characteristic A: uuid-1111 (Device Info)
#   Characteristic B: uuid-2222 (Commands)
#   Characteristic C: uuid-3333 ← THIS ONE SENDS TOKEN!
```

### Test 2: Token Extraction Only
```bash
# Extract token without logging in
async def test_token_only():
    listener = TokenListener("C6:1E:0D:XX:XX:XX")
    
    # Wake camera first
    ble_mgr = BLEManager()
    ble_mgr.wake_camera("C6:1E:0D:XX:XX:XX")
    await asyncio.sleep(1)
    
    # Extract token
    creds = await listener.listen(timeout=10)
    print(f"Token: {creds['token']}")
    print(f"Sequence: {creds['sequence'].hex()}")
    
    # Verify token length
    assert len(creds['token']) == 45, f"Token length {len(creds['token'])} != 45"
    print("✅ Token extraction works!")

asyncio.run(test_token_only())
```

### Test 3: Full Workflow
```bash
# Complete integration test
python3 src/main.py

# Expected output:
# ============================================================
# PHASE 1: BLE WAKE
# ============================================================
# [camera_mac] wakening...
# ✅ Wake successful
#
# ============================================================
# PHASE 2: TOKEN EXTRACTION
# ============================================================
# Listening for BLE notification...
# ✅ Token extracted: I3mbwVIxJQgnSB9...
# ✅ Sequence: 2b000000
#
# ============================================================
# PHASE 3: UDP LOGIN
# ============================================================
# Sending login with extracted token...
# ✅ LOGIN SUCCESSFUL!
# 🎉 COMPLETE SUCCESS! Camera authenticated!
```

### Test 4: Error Cases
```python
# Test missing token
camera = CameraClient("192.168.43.1")
try:
    camera.login()  # No set_session_credentials!
except ValueError:
    print("✅ Correctly rejects login without token")

# Test wrong token length
camera.set_session_credentials("short", b'\x2b\x00\x00\x00')
camera.login()  # Should get warning but attempt
# Expected: ⚠️  Token length warning

# Test wrong sequence
camera.set_session_credentials(
    "I3mbwVIxJQgnSB9GJKNk5Bvv/y+g8+MX/HVCMnCqyUo=",
    b'\xFF\x00\x00\x00'  # Invalid sequence
)
camera.login()  # Will likely fail, which is expected
```

---

## 📦 DELIVERABLES FOR JULES

### Code Files to Create
1. **ble_token_listener.py** - Token extraction from BLE
2. **ble_characteristic_scanner.py** - Discover token characteristic
3. **camera_client.py** (UPDATED) - Use dynamic token
4. **main.py** (UPDATED) - Full orchestration
5. **test_camera_login.py** - Test variants
6. **main_with_retries.py** - Production-ready version

### Documentation Files to Create
1. **BLE_TOKEN_DISCOVERY.md** - How to find token characteristic
2. **ARTEMIS_PROTOCOL.md** - Complete protocol specification
3. **TROUBLESHOOTING.md** - Common issues and solutions

### Configuration
1. Add to `config.yaml`:
```yaml
ble:
  token_characteristic_uuid: "???? (DISCOVER THIS!)"
  token_extraction_timeout: 10  # seconds
  token_validation:
    expected_length: 45
    expected_format: "base64"

camera:
  login_timeout: 5  # seconds
  login_retries: 3
  retry_delay: 2    # seconds
```

---

## 🎯 JULES PROMPT (COPY-PASTE THIS)

```
You are an expert Python developer specializing in BLE (Bluetooth Low Energy) 
protocol implementation and embedded systems.

TASK: Implement BLE Token Extraction for Camera Authentication

CONTEXT:
- Raspberry Pi needs to authenticate with a Reolink camera via UDP
- Current implementation uses a hardcoded token that never changes
- Real camera sends a NEW token via BLE notification after every wake cycle
- Token format: Base64 string, 45 characters
- Token must be extracted before attempting UDP login

TECHNICAL BACKGROUND:
The camera uses ARTEMIS binary protocol:
- Magic header: 0xf1d0
- Protocol string: "ARTEMIS\x00"
- Structure includes: Magic + Length + Header + Version + 
  DynamicSequence + TokenLength + Token

Real working example from TCP dump:
```
f1d0 0045 d100 0005 4152 5445 4d49 5300 0200 0000
2b00 0000 2d00 0000 49...
Magic Length  Proto  Version Seq   TokLen  Token...
```

REQUIREMENTS:
1. Create TokenListener class that:
   - Connects to camera via BLE after wake
   - Listens for BLE notification from specific characteristic
   - Parses token and sequence bytes from notification
   - Returns dict with {"token": "...", "sequence": b'...'}

2. Discover which BLE characteristic sends the token:
   - Create BLE scanner that lists all characteristics
   - Document UUID that sends token notification
   - Add this UUID to TokenListener hardcoded

3. Update CameraClient to:
   - Accept set_session_credentials(token, sequence)
   - Use dynamic token instead of hardcoded value
   - Build correct ARTEMIS packet with extracted values
   - Handle token not set error gracefully

4. Update main.py with complete workflow:
   - Phase 1: BLE Wake
   - Phase 2: Token Extraction (10 second timeout)
   - Phase 3: UDP Login with extracted token
   - Phase 4: Validate login success

5. Error handling for:
   - Token extraction timeout
   - BLE connection failure
   - Token format validation
   - Wrong token length warning

DELIVERABLES:
- ble_token_listener.py with TokenListener class
- ble_characteristic_scanner.py tool
- camera_client.py updated with dynamic token
- main.py orchestrating full workflow
- test_camera_login.py with test variants
- Documentation of discovered characteristic UUID

SUCCESS CRITERIA:
- Token extraction works (prints "I3mbwVIx..." format, 45 chars)
- Camera responds with LOGIN OK instead of TIMEOUT
- Sequence byte increments correctly on retry
- No hardcoded tokens in login flow

LOGGING:
- Add detailed debug logging at each step
- Log extracted token (first 20 chars + ...) not full token
- Log sequence bytes in hex format
- Print success/failure clearly
```

---

## 📊 SUCCESS METRICS

| Metric | Current | Target | Status |
|--------|---------|--------|--------|
| Token Extraction | ❌ Not implemented | ✅ Async listener | TODO |
| Token Length | 25 chars (wrong) | 45 chars | TODO |
| Login Success | ❌ TIMEOUT | ✅ OK | TODO |
| Sequence Dynamic | ❌ Static (02 00 01 00) | ✅ From BLE (2b/2c/2d) | TODO |
| Error Handling | ❌ Minimal | ✅ Comprehensive | TODO |
| Documentation | ❌ Incomplete | ✅ Complete | TODO |

---

## 🚀 TIMELINE

- **Hour 1**: BLE token listener + characteristic discovery
- **Hour 2**: Camera client updates + orchestration
- **Hour 3**: Testing + error handling refinement
- **Hour 3.5**: Documentation + final validation

**Total: ~3.5 hours of focused development**

---

## 💡 KEY INSIGHTS

1. **The token changes every time** → Must extract from BLE, not hardcode
2. **The sequence byte also changes** → Extracted from BLE too
3. **ARTEMIS wraps everything** → JSON, tokens, all go inside binary protocol
4. **Android app does this correctly** → Your smartphone logs show why it works
5. **The 0x2b/0x2c bytes were mysterious** → They're the BLE-provided sequence!

---

**Last Updated**: 2025-12-06  
**Next Review**: After Jules implementation  
**Owner**: @philibertschlutzki (you)


# 🎯 OPTIMIZED PROMPT FOR JULES AI

## Copy-Paste This Into Jules

---

```
You are an expert Python developer specializing in Bluetooth Low Energy (BLE)
protocol implementation, asynchronous programming, and embedded systems.

TASK: Implement Dynamic BLE Token Extraction for Camera Authentication

STATUS: Critical blocker - camera authentication fails because token is hardcoded

=============================================================================
BACKGROUND & CONTEXT
=============================================================================

The Problem:
- Raspberry Pi tries to authenticate with Reolink camera via UDP
- Uses hardcoded auth token: "MzlB36X/IVo8ZzI5rG9j1w==" (25 chars, WRONG)
- Result: Camera responds with TIMEOUT because token is stale
- Root cause: Token CHANGES on every BLE wake cycle, NOT permanent

What Works (Android App):
1. App wakes camera via BLE
2. Camera sends NEW token via BLE notification (45 chars)
3. App extracts token from BLE
4. App sends UDP login with EXTRACTED token
5. Camera responds: LOGIN OK ✓

What's Broken (Your Code):
1. Sends BLE wake ✓
2. IGNORES BLE notification response ✗
3. Uses hardcoded stale token ✗
4. Sends UDP login
5. Camera responds: TIMEOUT/AUTH FAILED ✗

=============================================================================
TECHNICAL SPECIFICATION
=============================================================================

The Token:
- Format: Base64 string
- Length: 45 characters (NOT 25!)
- Example: "I3mbwVIxJQgnSB9GJKNk5Bvv/y+g8+MX/HVCMnCqyUo="
- Source: BLE notification after wake
- Lifetime: Per-session (changes on next wake)

The Sequence Bytes:
- Also comes from BLE notification
- 4 bytes (e.g., 0x2b 0x00 0x00 0x00)
- Increments on retry (0x2b → 0x2c → 0x2d)
- Must be extracted and used, NOT hardcoded

The ARTEMIS Protocol (Binary Wrapper):
- Magic: 0xf1d0 (identifies ARTEMIS packet)
- Structure:
  * Bytes 0-1: 0xf1d0 (magic)
  * Bytes 2-3: payload length (big-endian)
  * Bytes 4-5: 0xd100 (constant header)
  * Bytes 6-7: 0x0005 (constant)
  * Bytes 8-17: "ARTEMIS\x00" (protocol string)
  * Bytes 18-25: 0x02000000 (version)
  * Bytes 26-29: SEQUENCE BYTES FROM BLE (CRITICAL!)
  * Bytes 30-33: Token length as 4-byte little-endian int
  * Bytes 34+: Token as ASCII + null terminator

Real Example (from TCP dump):
f1d0 0045 d100 0005 4152 5445 4d49 5300 0200 0000
2b00 0000 2d00 0000 49336d6277 56497846516 67... 
↑    ↑    ↑    ↑    ↑ARTEMIS      ↑version ↑seq ↑len    ↑token

=============================================================================
REQUIREMENTS - IMPLEMENT THESE COMPONENTS
=============================================================================

✅ COMPONENT 1: ble_token_listener.py

Create TokenListener class:

class TokenListener:
    def __init__(self, device_mac: str, logger=None)
    
    async def listen(self, timeout=10) -> dict:
        """
        Listen for BLE notification containing auth token.
        
        Returns:
            {"token": "I3mbwVIx...", "sequence": b'\x2b\x00\x00\x00'}
        
        Raises:
            asyncio.TimeoutError: If no notification within timeout
            BleakError: If BLE connection fails
        """
    
    Requirements:
    - Use bleak library (already installed)
    - Connect to camera MAC address
    - Register for BLE notifications
    - Parse token and sequence from notification payload
    - Timeout after N seconds
    - Log all operations at DEBUG level
    - Return dict with "token" (str) and "sequence" (bytes)
    
    Token Parsing Logic:
    - Notification payload structure (educated guess from behavior):
      [4 bytes: token_length] [4 bytes: sequence] [N bytes: token]
      Example: 2d 00 00 00 | 2b 00 00 00 | I3mbwVIx...
    - Extract token_length (little-endian) = 0x2d = 45 bytes
    - Extract sequence = next 4 bytes
    - Extract token = next 45 bytes, decode as ASCII, strip nulls
    
    Error Handling:
    - Timeout: log "Token extraction timeout", raise
    - BLE errors: log error, raise
    - Parse errors: log, raise with context


✅ COMPONENT 2: ble_characteristic_scanner.py

Create standalone script:

async def discover_token_characteristic(device_mac: str):
    """
    Scan camera and list all BLE characteristics.
    
    Purpose: Find which characteristic UUID sends the token.
    This is a discovery tool - run once to find token characteristic.
    
    Output:
    
    Service: 00000000-0000-0000-0000-000000000001
      Characteristic 1: UUID-1111 (can read, can notify)
      Characteristic 2: UUID-2222 (can write)
      Characteristic 3: UUID-3333 (can notify) ← TOKEN COMES HERE!
    
    Next step: Add UUID-3333 to TokenListener.NOTIFICATION_CHAR_UUID
    """
    
    Requirements:
    - Connect to device_mac
    - Discover all services
    - For each service, list characteristics
    - Print UUID, properties (read, write, notify, indicate)
    - Output format: Easy to identify which char sends notification


✅ COMPONENT 3: camera_client.py (UPDATE EXISTING)

Modify CameraClient class:

class CameraClient:
    def __init__(self, camera_ip: str, logger=None):
        self.camera_ip = camera_ip
        self.logger = logger or logging.getLogger(__name__)
        self.session_token = None  # ← NEW
        self.sequence_bytes = None # ← NEW
    
    def set_session_credentials(self, token: str, sequence: bytes):
        """
        Set auth credentials extracted from BLE.
        
        Args:
            token: Base64 string, 45 characters
            sequence: 4 bytes from BLE (e.g., b'\x2b\x00\x00\x00')
        
        Validation:
        - Warn if token length != 45
        - Log token (first 20 chars + "...")
        - Log sequence in hex
        """
    
    def _build_login_payload(self) -> bytes:
        """
        Build ARTEMIS binary login packet.
        
        Uses extracted token + sequence, NOT hardcoded values!
        
        Structure:
        - Magic: 0xf1d0
        - Length: calculated from payload
        - Header fields: 0xd100, 0x0005
        - Protocol: "ARTEMIS\x00"
        - Version: 0x02000000
        - Sequence: from BLE (not hardcoded!)
        - Token length: 0x2d000000 (little-endian 45)
        - Token: extracted token + null terminator
        
        Returns:
            bytes: Complete ARTEMIS packet
        
        Error:
            Raise ValueError if token/sequence not set
        """
        if not self.session_token or not self.sequence_bytes:
            raise ValueError(
                "Session credentials not set! "
                "Call set_session_credentials(token, sequence) first."
            )
        
        # Build packet structure
        # ... (exact structure detailed in BLE_TOKEN_STRATEGY.md)
    
    def login(self) -> bool:
        """
        Authenticate using extracted BLE token.
        
        Returns:
            True if login succeeds (camera responds)
            False if timeout or error
        """
        if not self.session_token:
            self.logger.error("No session token set!")
            return False
        
        try:
            payload = self._build_login_payload()
            response = self._send_udp(...)
            
            if response:
                self.logger.info("✓ LOGIN SUCCESSFUL")
                return True
            else:
                self.logger.error("✗ LOGIN FAILED - No response")
                return False
        except Exception as e:
            self.logger.error(f"✗ LOGIN ERROR: {e}")
            return False

    # KEEP EXISTING: _send_udp(), other methods
    # REMOVE: Hardcoded token constants


✅ COMPONENT 4: main.py (UPDATE TO ORCHESTRATE)

Import TokenListener and use in main workflow:

async def main():
    """
    Complete workflow with 3 phases.
    """
    CAMERA_MAC = "C6:1E:0D:XX:XX:XX"  # Replace
    CAMERA_IP = "192.168.43.1"
    
    try:
        # PHASE 1: BLE WAKE
        logger.info("="*60)
        logger.info("PHASE 1: BLE WAKE")
        logger.info("="*60)
        
        ble_mgr = BLEManager()
        ble_mgr.wake_camera(CAMERA_MAC)
        await asyncio.sleep(2)
        
        # PHASE 2: TOKEN EXTRACTION (NEW!)
        logger.info("="*60)
        logger.info("PHASE 2: TOKEN EXTRACTION")
        logger.info("="*60)
        
        token_listener = TokenListener(CAMERA_MAC, logger)
        creds = await token_listener.listen(timeout=10)
        
        logger.info(f"✓ Token: {creds['token'][:20]}...")
        logger.info(f"✓ Sequence: {creds['sequence'].hex()}")
        
        # PHASE 3: UDP LOGIN
        logger.info("="*60)
        logger.info("PHASE 3: UDP LOGIN")
        logger.info("="*60)
        
        camera = CameraClient(CAMERA_IP, logger)
        camera.set_session_credentials(creds['token'], creds['sequence'])
        
        if camera.login():
            logger.info("🎉 AUTHENTICATION SUCCESSFUL!")
            return True
        else:
            logger.error("✗ Login failed despite correct token")
            return False
    
    except asyncio.TimeoutError:
        logger.error("✗ Token extraction timeout")
        return False
    except Exception as e:
        logger.error(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


✅ COMPONENT 5: test_camera_login.py (CREATE TEST FILE)

Test class with variants:

class TestCameraLogin:
    """
    Test different authentication scenarios.
    """
    
    def test_extracted_token(self, token: str, sequence: bytes):
        """Test with real BLE-extracted token (should work)."""
        # Expected: SUCCESS
    
    def test_hardcoded_token(self):
        """Test with hardcoded stale token (should fail)."""
        # Expected: TIMEOUT
    
    def test_sequence_increment(self, base_token, base_seq):
        """Test sequence increment on retry (should work)."""
        # Increment sequence and retry
        # Expected: SUCCESS


=============================================================================
IMPLEMENTATION CHECKLIST
=============================================================================

☐ Create ble_token_listener.py:
  ☐ TokenListener class with async listen()
  ☐ Parse token and sequence from BLE notification
  ☐ Comprehensive error handling
  ☐ Debug logging at each step

☐ Create ble_characteristic_scanner.py:
  ☐ Discover all BLE characteristics
  ☐ Print characteristic UUIDs and properties
  ☐ Output format easy to identify token source
  ☐ Runnable as standalone script

☐ Update camera_client.py:
  ☐ Add set_session_credentials() method
  ☐ Update _build_login_payload() to use dynamic token
  ☐ Remove hardcoded token references
  ☐ Add validation for token/sequence
  ☐ Update login() to use set values

☐ Update main.py:
  ☐ Import TokenListener
  ☐ Add Phase 2: Token extraction
  ☐ Pass extracted token to CameraClient
  ☐ Clear logging output for each phase

☐ Create test_camera_login.py:
  ☐ Test with extracted token (should work)
  ☐ Test with hardcoded token (should fail)
  ☐ Test sequence increment

☐ Documentation:
  ☐ Update README with new workflow
  ☐ Document discovered characteristic UUID
  ☐ Add troubleshooting guide


=============================================================================
SUCCESS CRITERIA
=============================================================================

✓ Token extraction:
  - Prints token like "I3mbwVIx..." (45 chars)
  - Not "MzlB36X..." (old 25 char hardcoded)
  - Completes within 10 seconds
  - Shows sequence bytes in hex

✓ Camera authentication:
  - Camera responds with LOGIN OK (not TIMEOUT)
  - Works on first attempt
  - Works on retries with incremented sequence
  - No hardcoded tokens in login code

✓ Error handling:
  - Timeout if BLE doesn't respond
  - Clear error messages
  - Suggests corrective actions
  - Token validation warnings

✓ Code quality:
  - Type hints on all functions
  - Comprehensive docstrings
  - Debug logging throughout
  - No bare except clauses


=============================================================================
KEY INSIGHTS (READ THESE!)
=============================================================================

1. The token is NOT permanent!
   - Old: Thought token was static
   - Correct: Token changes on every wake cycle
   - Action: Must extract from BLE, not hardcode

2. The sequence bytes are also dynamic!
   - Old: Hardcoded 0x02000100
   - Correct: Comes from BLE (0x2b, 0x2c, etc.)
   - Action: Extract from same BLE notification as token

3. ARTEMIS is a binary wrapper, not replacement for JSON!
   - Old: Confused about "JSON protocol"
   - Correct: Token gets wrapped in ARTEMIS binary
   - Action: Build correct packet structure

4. Android app does this correctly!
   - Shows: Wake → Extract → Login
   - Your code: Wake → Hardcode → Timeout
   - Lesson: Reverse-engineer from what works

5. The 0x2b/0x2c mystery is solved!
   - These bytes come from BLE, not random
   - They increment on retry
   - They're part of the session state


=============================================================================
TIMELINE
=============================================================================

Estimated effort: 3-4 hours
- TokenListener + discovery: 60-90 min
- CameraClient updates: 45-60 min
- Integration + testing: 45-60 min
- Refinement + docs: 30-45 min

Start with TokenListener (most critical).
Test token extraction in isolation before touching camera_client.py.


=============================================================================
DEPENDENCIES & ENVIRONMENT
=============================================================================

Libraries (should be installed):
- bleak (BLE client)
- asyncio (async support)
- logging (already in stdlib)

Python version:
- 3.8+ (for async/await support)

Existing code to reuse:
- src/camera_client.py (update, don't rewrite)
- src/ble_manager.py (for wake_camera())
- Logging setup in existing main.py


=============================================================================
QUESTIONS TO RESOLVE DURING IMPLEMENTATION
=============================================================================

1. Which BLE characteristic sends the token?
   → Use ble_characteristic_scanner.py to find out
   → Document the UUID in TokenListener

2. Exact BLE notification payload format?
   → Current assumption: [length][sequence][token]
   → May need adjustment after first test
   → Log raw payload for debugging

3. What if token extraction times out?
   → Current design: Raise exception
   → Alternative: Use fallback (not recommended)
   → Decision: Fail fast, let caller decide retry

4. Token validation rules?
   → Must be 45 characters
   → Must be valid Base64 (decodable)
   → Should warn if length mismatches


=============================================================================
DEBUG LOGGING TEMPLATE
=============================================================================

Add these log statements for visibility:

logger.debug(f"Characteristic discovered: {uuid}")
logger.debug(f"Raw BLE payload: {raw_data.hex()}")
logger.debug(f"Token length field: {token_len}")
logger.debug(f"Sequence bytes: {sequence.hex()}")
logger.info(f"✓ Token extracted: {token[:20]}... (len={len(token)})")
logger.warning(f"Token length {len(token)} != 45")
logger.error(f"Token extraction failed: {error}")


=============================================================================
FINAL NOTES
=============================================================================

- This is a high-confidence implementation plan
- Root cause is well understood from TCP dumps
- Success criteria are specific and measurable
- Each component has a clear purpose
- Testing will validate the approach
- Document discoveries (especially UUID) for future reference

Good luck! This should fix the authentication issue completely.
```

---

## Usage

1. Copy the entire code block above
2. Paste into a conversation with Jules AI
3. Jules will implement all components automatically
4. Monitor progress and ask for clarifications as needed

## Expected Output from Jules

After 3-4 hours:

```
✓ ble_token_listener.py (200 lines)
✓ ble_characteristic_scanner.py (150 lines)
✓ camera_client.py updated (80 lines modified)
✓ main.py updated (50 lines modified)
✓ test_camera_login.py (120 lines)
✓ Documentation updated
```

Then:
1. Run characteristic scanner: `python3 ble_characteristic_scanner.py <MAC>`
2. Find token characteristic UUID
3. Update TokenListener with UUID
4. Run main.py: `python3 main.py`
5. Observe: "✓ LOGIN SUCCESSFUL!"

---

**Version**: 1.0  
**Status**: Ready for Jules  
**Owner**: @philibertschlutzki  
**Updated**: 2025-12-06

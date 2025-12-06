# 🛠️ IMPLEMENTATION PLAN - BLE Token Strategy

**Status**: Ready for Jules AI  
**Priority**: CRITICAL (Blocks camera authentication)  
**Estimated Effort**: 3-4 hours  
**Target Completion**: This session  

---

## 📄 CURRENT STATE

### What's Broken

```
Raspberry Pi → BLE Wake → (Camera: OK, sends new token)
             → Ignores token notification ✗
             → Uses hardcoded stale token ✗
             → UDP Login → TIMEOUT/AUTH_FAILED ✗
```

### Symptom
- `logger.error("timeout: No authentication received")` in logs
- Camera doesn't respond to login attempts
- Works on Android phone but not on RPi

### Root Cause
- **Hardcoded Token**: `"MzlB36X/IVo8ZzI5rG9j1w=="` (25 chars, wrong format)
- **Missing Extraction**: No code listens to BLE notification
- **Wrong Bytes**: Uses `0x02000100` instead of dynamic sequence from BLE
- **Protocol Misunderstanding**: Thought it was JSON, it's ARTEMIS binary

---

## 🌈 TARGET STATE

### Correct Workflow

```
Raspberry Pi → Phase 1: BLE Wake
             → Camera responds with new token in BLE notification
             → Phase 2: Extract Token & Sequence
             → Listen to BLE, parse: (45-char token, sequence bytes)
             → Phase 3: UDP Login with extracted values
             → Build ARTEMIS packet with DYNAMIC token + sequence
             → Camera responds: LOGIN OK ✅
```

### Expected Success Indicators
- Token extracted: `"I3mbwVIx..."` (45 chars, correct format)
- Sequence bytes: `2b 00 00 00` (from BLE, not hardcoded)
- Camera responds to login (not timeout)
- logger output: `"✅ LOGIN SUCCESSFUL"`

---

## 🟁 PHASE BREAKDOWN

### Phase 1: BLE Token Listener (CRITICAL)

**What**: Create new module to extract token from BLE

**File**: `src/ble_token_listener.py`

**Components**:
```python
class TokenListener:
    async def listen(timeout=10) -> dict
        Returns: {"token": str(45), "sequence": bytes(4)}
```

**Key Points**:
- Must discover which BLE characteristic sends token
- Parse token + sequence from notification payload
- Validate token is 45 chars
- Timeout handling (10 sec max)
- Debug logging every step

**Dependencies**: `bleak`, `asyncio`

**Testing**: Run standalone, verify token extraction

---

### Phase 2: BLE Characteristic Discovery (REQUIRED)

**What**: One-time tool to find token characteristic UUID

**File**: `src/ble_characteristic_scanner.py`

**Purpose**:
```bash
python3 ble_characteristic_scanner.py C6:1E:0D:XX:XX:XX

Output:
Service: 1800 (Generic Access)
  Char A: 2a00 (Device Name) - read, write
  Char B: 2a01 (Appearance) - read
Service: #### (Vendor Specific?)
  Char C: ???? (SENDS NOTIFICATION!) ← THIS ONE
  Char D: ???? (Receives data)
```

**Action After**:
- Document discovered UUID
- Add to TokenListener.NOTIFICATION_CHAR_UUID (hardcoded once)
- Commit documentation

---

### Phase 3: Camera Client Update (CORE CHANGE)

**What**: Modify to use dynamic token instead of hardcoded

**File**: `src/camera_client.py` (existing, modify)

**Changes**:

1. Add token storage:
   ```python
   self.session_token = None
   self.sequence_bytes = None
   ```

2. Add credential setter:
   ```python
   def set_session_credentials(self, token: str, sequence: bytes):
       # Validate and store
   ```

3. Update packet builder:
   ```python
   def _build_login_payload(self) -> bytes:
       # Use self.session_token (not hardcoded!)
       # Use self.sequence_bytes (from BLE!)
   ```

4. Remove hardcoded tokens

**Result**: All authentication uses dynamic values

---

### Phase 4: Main Orchestration (INTEGRATION)

**What**: Update main.py to orchestrate 3 phases

**File**: `src/main.py` (existing, modify)

**Changes**:

```python
async def main():
    # Phase 1: Wake
    ble_mgr.wake_camera(MAC)
    await asyncio.sleep(2)
    
    # Phase 2: Extract token (NEW!)
    listener = TokenListener(MAC)
    creds = await listener.listen(timeout=10)
    
    # Phase 3: Login
    camera = CameraClient(IP)
    camera.set_session_credentials(creds["token"], creds["sequence"])
    success = camera.login()
```

**Logging**: Clear phase separators, progress indicators

---

### Phase 5: Testing & Validation (VERIFICATION)

**What**: Test suite covering all scenarios

**File**: `src/test_camera_login.py` (new)

**Tests**:

1. **test_token_extraction**
   - Extract token
   - Verify length = 45
   - Verify format is Base64-like
   - Expected: PASS

2. **test_login_with_extracted**
   - Use real extracted token
   - Expected: LOGIN OK

3. **test_login_with_hardcoded**
   - Use old hardcoded token
   - Expected: TIMEOUT (proves it was wrong)

4. **test_sequence_increment**
   - Extract token
   - First login with sequence
   - Retry with incremented sequence
   - Expected: Both work

---

## 🐤 JULES AI WORKFLOW

### Step 1: Provide Context & Prompt

**Input Files**:
- `JULES_PROMPT.md` (ready to copy-paste)
- `BLE_TOKEN_STRATEGY.md` (reference material)
- Existing `src/camera_client.py` (for context)
- Existing `src/main.py` (for context)

**Process**:
1. Copy `JULES_PROMPT.md` content
2. Paste into Jules conversation
3. Jules reads prompt and analyzes requirements
4. Jules asks clarifying questions if needed

### Step 2: Implementation

**Component 1: ble_token_listener.py** (60-90 min)
- TokenListener class
- async listen() method
- BLE notification handling
- Token/sequence parsing
- Error handling
- Debug logging

**Component 2: ble_characteristic_scanner.py** (30-45 min)
- BLE discovery
- Service enumeration
- Characteristic listing
- Human-readable output
- Standalone script

**Component 3: camera_client.py update** (45-60 min)
- Add credential storage
- Add setter method
- Update packet builder
- Use dynamic values
- Remove hardcoded tokens
- Keep existing functionality

**Component 4: main.py update** (30-45 min)
- Import TokenListener
- Add Phase 2
- Wire up orchestration
- Clear logging
- Error handling

**Component 5: test_camera_login.py** (45-60 min)
- Test class
- 4+ test methods
- Scenarios covering
- Debug output

### Step 3: Testing & Discovery

**1. Run characteristic scanner** (5 min)
```bash
python3 src/ble_characteristic_scanner.py C6:1E:0D:XX:XX:XX
# Find token characteristic UUID
# Document in TokenListener
```

**2. Test token extraction** (10 min)
```bash
python3 -c "
from src.ble_token_listener import TokenListener
import asyncio

async def test():
    listener = TokenListener('C6:1E:0D:XX:XX:XX')
    # Wake first
    creds = await listener.listen()
    print(f'Token: {creds[\"token\"][:20]}...')
    print(f'Length: {len(creds[\"token\"])}')
    print(f'Sequence: {creds[\"sequence\"].hex()}')

asyncio.run(test())
"

# Expected:
# Token: I3mbwVIxJQgnSB9G...
# Length: 45
# Sequence: 2b000000
```

**3. Test full workflow** (15 min)
```bash
python3 src/main.py

# Expected output:
# ============================================================
# PHASE 1: BLE WAKE
# ============================================================
# [camera_mac] waking...
# ✅ Wake successful
#
# ============================================================
# PHASE 2: TOKEN EXTRACTION
# ============================================================
# Listening for BLE notification (timeout: 10s)...
# ✓ Token extracted: I3mbwVIx... (len=45)
# ✓ Sequence: 2b000000
#
# ============================================================
# PHASE 3: UDP LOGIN
# ============================================================
# Building ARTEMIS packet...
# Sending login packet...
# ✓ LOGIN SUCCESSFUL!
# 🎉 AUTHENTICATION COMPLETE!
```

**4. Run test suite** (10 min)
```bash
python3 src/test_camera_login.py

# Expected: All tests pass except hardcoded variant (expected to fail)
```

### Step 4: Documentation

**Update/Create**:
- `README.md`: New authentication workflow diagram
- `BLE_DISCOVERY_RESULTS.md`: Document discovered UUID + timestamp
- `TROUBLESHOOTING.md`: Common issues + fixes
- Code docstrings: Already included by Jules

---

## ✅ SUCCESS CRITERIA

### Technical Success
- [ ] Token extracted: Shows 45-character Base64 string
- [ ] Token changes: Different on each wake cycle
- [ ] Sequence extracted: Shows hex bytes (2b, 2c, 2d)
- [ ] Camera responds: LOGIN OK message appears
- [ ] No TIMEOUT: Authentication completes in <5 seconds
- [ ] Retries work: Sequence increments, login succeeds

### Code Quality
- [ ] Type hints on all functions
- [ ] Comprehensive docstrings
- [ ] Debug logging throughout
- [ ] No hardcoded tokens in critical path
- [ ] Error handling for all failure modes
- [ ] Tests covering all scenarios

### Integration
- [ ] Works with existing BLEManager
- [ ] No breaking changes to camera_client interface
- [ ] main.py runs without errors
- [ ] Logging shows clear progress

### Documentation
- [ ] BLE characteristic UUID documented
- [ ] Protocol structure explained in code
- [ ] Workflow diagrams updated
- [ ] Troubleshooting guide provided

---

## 🚁 TIMELINE & MILESTONES

```
Day 1 (Today):
⏰ Hour 0-1:
  ✔ Analyze requirements
  ✔ Create BLE token listener
  ✔ Design packet parsing
⏰ Hour 1-1.5:
  ✔ Create characteristic scanner
  ✔ Test BLE discovery
⏰ Hour 1.5-2.5:
  ✔ Update camera_client.py
  ✔ Add credential setter
  ✔ Update packet builder
⏰ Hour 2.5-3:
  ✔ Update main.py
  ✔ Wire orchestration
⏰ Hour 3-3.5:
  ✔ Create test suite
  ✔ Run tests
  ✔ Fix issues
⏰ Hour 3.5-4:
  ✔ Documentation
  ✔ Final validation
✅ TOTAL: 4 hours (includes testing & refinement)
```

---

## 🛡️ DEPENDENCIES & REQUIREMENTS

### Already Installed
- `bleak` (BLE client library)
- `asyncio` (async support)
- `logging` (stdlib)

### Environment
- Python 3.8+ (for async/await)
- Linux/RPi (for Bluetooth support)
- Camera must be accessible (on same network)

### Files to Provide Jules
1. `JULES_PROMPT.md` (specification)
2. `BLE_TOKEN_STRATEGY.md` (technical reference)
3. Existing `src/camera_client.py` (context)
4. Existing `src/main.py` (context)
5. Existing `src/ble_manager.py` (for wake_camera reference)

---

## 📢 HANDOFF TO JULES

### What You Do
1. Copy content of `JULES_PROMPT.md`
2. Open Jules AI conversation
3. Paste prompt as-is
4. Let Jules implement all components
5. Provide GitHub repo path when asked

### What Jules Does
1. Read and understand requirements
2. Ask clarifying questions if needed
3. Implement all 5 components
4. Write comprehensive docstrings
5. Create test file
6. Commit to repository

### What You Do After
1. Run characteristic scanner
2. Identify token characteristic UUID
3. Test token extraction
4. Run full workflow
5. Verify camera authentication works
6. Document discovered UUID
7. Commit findings

---

## 💪 KEY POINTS FOR JULES

Make sure Jules understands:

1. **Token is dynamic**
   - Not permanent, changes on every wake
   - 45 characters, Base64 format
   - Comes from BLE notification

2. **Sequence is also dynamic**
   - 4 bytes from BLE
   - Increments on retry (0x2b → 0x2c)
   - Part of ARTEMIS packet structure

3. **Protocol is ARTEMIS (binary)**
   - Not JSON over UDP
   - Binary wrapper with magic header
   - Token is payload inside wrapper

4. **Android app proves it works**
   - Wake → Extract token → Login = SUCCESS
   - Your code: Wake → Hardcode token → Timeout = FAILURE
   - Reverse-engineer from what works

5. **BLE characteristic UUID is unknown**
   - Need to discover using scanner tool
   - Document once found
   - Hardcode in TokenListener after discovery

---

## 🏃 EXECUTION CHECKLIST

### Pre-Jules
- [ ] Read this document fully
- [ ] Read `BLE_TOKEN_STRATEGY.md` for technical context
- [ ] Prepare `JULES_PROMPT.md` for copy-paste
- [ ] Have GitHub repo URL ready

### During Jules Implementation
- [ ] Monitor for clarifying questions
- [ ] Provide repo access if needed
- [ ] Don't interrupt implementation
- [ ] Note any issues Jules encounters

### Post-Implementation
- [ ] Check created files are correct
- [ ] Run characteristic scanner: `ble_characteristic_scanner.py`
- [ ] Document discovered UUID
- [ ] Test token extraction: `TokenListener.listen()`
- [ ] Run full workflow: `main.py`
- [ ] Verify camera authentication
- [ ] Commit findings
- [ ] Update documentation with real UUID

---

## 🚩 RISK MITIGATION

### Risk: Characteristic UUID not discovered
**Mitigation**: Fallback to brute-force notification listening

### Risk: Token parsing fails
**Mitigation**: Log raw BLE payload, adjust parsing logic

### Risk: Sequence format unexpected
**Mitigation**: Try different byte orders (little-endian vs big-endian)

### Risk: Timeout on token extraction
**Mitigation**: Increase timeout, verify BLE wake successful

### Risk: Login still fails despite correct token
**Mitigation**: Check packet structure matches exactly, debug logs

---

## 🃚 REFERENCES

- `BLE_TOKEN_STRATEGY.md` - Complete technical analysis
- `JULES_PROMPT.md` - Exact prompt for Jules
- TCP dump analysis - Shows real ARTEMIS packets
- Android app logs - Shows correct workflow

---

**Status**: Ready for Jules AI  
**Last Updated**: 2025-12-06 21:30 UTC  
**Owner**: @philibertschlutzki  
**Next**: Copy JULES_PROMPT.md and provide to Jules AI

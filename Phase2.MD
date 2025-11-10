# Phase 2: Hands with Piano

**Goal:** Verify hands can interact with piano keys to produce sound.

---

## Step 2.1: Hand Model Tests

Run `shadow_hand_test.py` to verify:
- Hand compiles and steps correctly
- Both left and right hands work
- All joints and actuators are present
- Fingertip sites are correctly positioned
- Touch sensors work (if applicable)

---

## Step 2.2: Hand-Piano Collision Setup

Test collision detection:
- Verify hand fingertips can contact piano keys
- Test collision geometry is correct
- Verify keys respond to hand contact
- Check for penetration or collision issues

---

## Step 2.3: Hand Positioning

Test hand placement:
- Position hands above piano keys
- Verify hands can reach all key ranges
- Test both left and right hand positions
- Verify hand orientation is correct for key pressing

---

## Step 2.4: Manual Key Pressing

Test hand pressing keys:
- Move hand to press a single key
- Verify key activates when pressed by hand
- Test key release when hand moves away
- Verify MIDI messages are generated
- Test multiple keys pressed simultaneously

---

## Step 2.5: Hand Control Integration

Test hand control:
- Apply actions to hand joints
- Verify hand moves fingers to press keys
- Test coordinated finger movements
- Verify hand can play simple sequences

---

## Step 2.6: Full Environment Test

Run `piano_with_shadow_hands_env.py`:
- Load environment with both hands and piano
- Verify all components initialize correctly
- Test hand actions affect piano keys
- Verify MIDI generation works with hand interaction
- Test playing a simple melody with hands

---

## Step 2.7: Dual Hand Coordination

Test both hands together:
- Left hand plays bass notes
- Right hand plays melody
- Verify no collisions between hands
- Test simultaneous key presses from both hands
- Verify MIDI messages from both hands

---

## Step 2.8: Hand-Piano Interaction Validation

Comprehensive interaction tests:
- Verify hand can press keys with correct force
- Test key activation threshold is appropriate
- Verify keys return to rest when hand releases
- Test hand can play chords (multiple keys)
- Verify no physics instabilities during interaction

---

## Success Criteria for Phase 2

- ✅ Hand models load and compile correctly
- ✅ Hands can be positioned above piano keys
- ✅ Hand fingertips can contact and press keys
- ✅ Keys activate when pressed by hands
- ✅ MIDI messages generated from hand interaction
- ✅ Both hands can work independently
- ✅ Hands can play simple melodies
- ✅ No collision or physics issues

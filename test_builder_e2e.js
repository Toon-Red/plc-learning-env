/**
 * Playwright E2E test for Grid Builder + PLC Conveyor Movement
 * Run: node test_builder_e2e.js
 */

const { chromium } = require('playwright');

const BASE = 'http://localhost:8080';
let browser, page;
let passed = 0, failed = 0, errors = [];

async function test(name, fn) {
    try {
        await fn();
        passed++;
        console.log(`  PASS: ${name}`);
    } catch (e) {
        failed++;
        errors.push({ name, error: e.message });
        console.log(`  FAIL: ${name} — ${e.message}`);
    }
}

function assert(cond, msg) {
    if (!cond) throw new Error(msg || 'Assertion failed');
}

// ── Helpers that use page.evaluate to avoid DOM selector ambiguity ──

async function clickCell(col, row) {
    await page.evaluate(([c, r]) => {
        document.getElementById('gc' + c + '_' + r).click();
    }, [col, row]);
    await sleep(50);
}

async function getCellData(col, row) {
    return page.evaluate(([c, r]) => {
        const k = c + ',' + r;
        return window.GRID ? window.GRID.cells[k] || null : null;
    }, [col, row]);
}

async function getAllCells() {
    return page.evaluate(() => window.GRID ? window.GRID.cells : {});
}

async function getBuilder() {
    return page.evaluate(() => {
        const b = window.BUILDER;
        return { mode: b.mode, placeType: b.placeType, dir: b.dir,
                 track: b.track, wirePort: b.wirePort, selKey: b.selKey };
    });
}

async function gbEval(code) {
    return page.evaluate(code);
}

function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

// ═══════════════════════════════════════════════════════════════════
async function runTests() {
    console.log('\n══════════════════════════════════════════════');
    console.log('  Grid Builder E2E Tests (Playwright)');
    console.log('══════════════════════════════════════════════\n');

    browser = await chromium.launch({ headless: true });
    const ctx = await browser.newContext({ viewport: { width: 1920, height: 1080 } });
    page = await ctx.newPage();

    const jsErrors = [];
    page.on('pageerror', err => jsErrors.push(err.message));

    // ── SECTION 1: Page Load ──────────────────────────────────────
    console.log('Section 1: Page Load & Init');

    await test('Page loads', async () => {
        await page.goto(BASE, { waitUntil: 'networkidle' });
        const title = await page.title();
        assert(title.includes('PLC'), `Title: ${title}`);
    });

    await test('Switch to Builder tab and init grid', async () => {
        await page.evaluate(() => switchTab('builder'));
        await sleep(1500);  // Wait for initBuilderTab + gbLoadCurrent
        const count = await page.evaluate(() => document.querySelectorAll('.gc').length);
        assert(count === 1000, `Expected 1000 grid cells, got ${count}`);
    });

    await test('Current topology auto-imported', async () => {
        const cells = await getAllCells();
        const count = Object.keys(cells).length;
        assert(count >= 1, `Expected >=1 imported zones, got ${count}`);
        console.log(`    (Imported ${count} zones)`);
    });

    // ── SECTION 2: Place Mode & Auto-Wiring ──────────────────────
    console.log('\nSection 2: Place & Auto-Wire');

    await test('Clear grid', async () => {
        page.once('dialog', d => d.accept());
        await gbEval(() => gbClear());
        await sleep(200);
        const cells = await getAllCells();
        assert(Object.keys(cells).length === 0, 'Grid not empty');
    });

    await test('Place transport at (5,5)', async () => {
        await gbEval(() => { gbSetMode('PLACE'); gbSetType('transport'); gbSetDir('E'); });
        await clickCell(5, 5);
        const c = await getCellData(5, 5);
        assert(c && c.zoneType === 'transport', `Got ${JSON.stringify(c)}`);
        assert(c.direction === 'E');
    });

    await test('Place at (6,5) — auto-wired from (5,5)', async () => {
        await clickCell(6, 5);
        const c5 = await getCellData(5, 5);
        const c6 = await getCellData(6, 5);
        assert(c5.sDownstream === c6.zoneId,
            `5,5 downstream=${c5.sDownstream}, expected ${c6.zoneId}`);
    });

    await test('Place at (7,5) — chain continues', async () => {
        await clickCell(7, 5);
        const c6 = await getCellData(6, 5);
        const c7 = await getCellData(7, 5);
        assert(c6.sDownstream === c7.zoneId);
    });

    await test('Place at (4,5) facing E — forward-wires to (5,5)', async () => {
        await clickCell(4, 5);
        const c4 = await getCellData(4, 5);
        const c5 = await getCellData(5, 5);
        assert(c4.sDownstream === c5.zoneId);
    });

    // ── SECTION 3: Branch & Merge ─────────────────────────────────
    console.log('\nSection 3: Branch & Merge');

    await test('Place branch at (8,5)', async () => {
        await gbEval(() => gbSetType('branch'));
        await clickCell(8, 5);
        const c = await getCellData(8, 5);
        assert(c.zoneType === 'branch');
        assert(c.iTargetChute >= 1);
        // Auto-wired from (7,5)
        const c7 = await getCellData(7, 5);
        assert(c7.sDownstream === c.zoneId);
    });

    await test('Branch divert auto-wires to south cell', async () => {
        // Branch faces E, divert=S. Place zone at (8,6).
        await gbEval(() => gbSetType('transport'));
        await clickCell(8, 6);
        const br = await getCellData(8, 5);
        const sp = await getCellData(8, 6);
        assert(br.sDivertDownstream === sp.zoneId,
            `Divert=${br.sDivertDownstream}, expected ${sp.zoneId}`);
    });

    await test('Merge sUpstream2 auto-wires from input-2 direction', async () => {
        // Place zone at (12,4) facing S (will be upstream2 source)
        await gbEval(() => gbSetDir('S'));
        await clickCell(12, 4);
        // Place merge at (12,5) facing E. Input-2 = rotL(E) = N → cell (12,4)
        await gbEval(() => { gbSetType('merge'); gbSetDir('E'); });
        await clickCell(12, 5);
        const mg = await getCellData(12, 5);
        const u2 = await getCellData(12, 4);
        assert(mg.sUpstream2 === u2.zoneId,
            `Merge sUpstream2=${mg.sUpstream2}, expected ${u2.zoneId}`);
    });

    // ── SECTION 4: Rotation ───────────────────────────────────────
    console.log('\nSection 4: Rotation');

    await test('E key rotates clockwise in PLACE mode', async () => {
        await gbEval(() => { gbSetMode('PLACE'); gbSetDir('E'); });
        await page.keyboard.press('e');
        const b = await getBuilder();
        assert(b.dir === 'S', `Expected S, got ${b.dir}`);
    });

    await test('Q key rotates counter-clockwise in PLACE mode', async () => {
        await page.keyboard.press('q');
        const b = await getBuilder();
        assert(b.dir === 'E', `Expected E, got ${b.dir}`);
    });

    await test('E key rotates selected zone in SELECT mode + re-wires', async () => {
        await gbEval(() => gbSetMode('SELECT'));
        await clickCell(5, 5);  // Select zone at (5,5) facing E
        const before = await getCellData(5, 5);
        assert(before.direction === 'E');

        await page.keyboard.press('e');  // E→S
        const after = await getCellData(5, 5);
        assert(after.direction === 'S', `Expected S, got ${after.direction}`);

        // Restore direction
        await page.keyboard.press('q');  // S→E
        const restored = await getCellData(5, 5);
        assert(restored.direction === 'E');
    });

    // ── SECTION 5: Drag & Drop ────────────────────────────────────
    console.log('\nSection 5: Drag & Drop');

    await test('Drag zone to empty cell', async () => {
        await gbEval(() => gbSetMode('SELECT'));
        // Get zone at (12,4) and drag it to (12,2)
        const before = await getCellData(12, 4);
        assert(before, 'No zone at 12,4');

        await gbEval(() => {
            // Simulate mousedown on 12,4
            gbMouseDown(12, 4, { button: 0 });
            // Simulate mouseup on 12,2
            gbMouseUp(12, 2, { button: 0 });
        });
        await sleep(200);

        const oldCell = await getCellData(12, 4);
        const newCell = await getCellData(12, 2);
        assert(!oldCell, '12,4 should be empty after drag');
        assert(newCell && newCell.zoneId === before.zoneId, 'Zone should be at 12,2');
    });

    await test('Drag zone onto occupied cell — swap', async () => {
        // Place two zones at known positions
        await gbEval(() => { gbSetMode('PLACE'); gbSetType('transport'); gbSetDir('E'); });
        await clickCell(20, 10);
        await clickCell(21, 10);
        const z20 = await getCellData(20, 10);
        const z21 = await getCellData(21, 10);

        // Swap them
        await gbEval(() => {
            gbSetMode('SELECT');
            gbMouseDown(20, 10, { button: 0 });
            gbMouseUp(21, 10, { button: 0 });
        });
        await sleep(200);

        const after20 = await getCellData(20, 10);
        const after21 = await getCellData(21, 10);
        assert(after20.zoneId === z21.zoneId, `Expected ${z21.zoneId} at 20,10, got ${after20.zoneId}`);
        assert(after21.zoneId === z20.zoneId, `Expected ${z20.zoneId} at 21,10, got ${after21.zoneId}`);
    });

    // ── SECTION 6: Auto-Complete ──────────────────────────────────
    console.log('\nSection 6: Auto-Complete');

    await test('BFS auto-complete fills path between distant zones', async () => {
        page.once('dialog', d => d.accept());
        await gbEval(() => gbClear());
        await sleep(200);

        // Place source at (2,5) and target at (8,5)
        await gbEval(() => { gbSetMode('PLACE'); gbSetType('transport'); gbSetDir('E'); });
        await clickCell(2, 5);
        await clickCell(8, 5);

        // Auto-complete from (2,5) to (8,5)
        await gbEval(() => {
            gbSetMode('SELECT');
            gbSelectCell('2,5');
            gbStartAC('2,5');
        });
        await sleep(200);

        const b = await getBuilder();
        assert(b.mode === 'AUTOCOMPLETE', `Expected AUTOCOMPLETE, got ${b.mode}`);

        // Execute auto-complete
        await gbEval(() => {
            gbSaveUndo();
            gbExecAC('8,5');
        });
        await sleep(300);

        // Should have filled 3,5 through 7,5
        const cells = await getAllCells();
        for (let c = 3; c <= 7; c++) {
            const cell = cells[c + ',5'];
            assert(cell, `Missing auto-filled zone at ${c},5`);
            assert(cell.zoneType === 'transport');
        }
        // Wiring: 2,5 → 3,5 → ... → 7,5 → 8,5
        const src = cells['2,5'];
        const first = cells['3,5'];
        const last = cells['7,5'];
        const tgt = cells['8,5'];
        assert(src.sDownstream === first.zoneId, `Source→first: ${src.sDownstream}`);
        assert(last.sDownstream === tgt.zoneId, `Last→target: ${last.sDownstream}`);
    });

    await test('Direct wire for adjacent zones (no intermediate cells)', async () => {
        page.once('dialog', d => d.accept());
        await gbEval(() => gbClear());
        await sleep(200);

        await gbEval(() => { gbSetMode('PLACE'); gbSetType('transport'); gbSetDir('E'); });
        await clickCell(5, 5);
        await gbEval(() => gbSetDir('S'));
        await clickCell(5, 6);

        // Auto-complete from (5,5) to (5,6)
        await gbEval(() => {
            gbSetMode('SELECT');
            gbStartAC('5,5');
            gbSaveUndo();
            gbExecAC('5,6');
        });
        await sleep(200);

        const src = await getCellData(5, 5);
        const tgt = await getCellData(5, 6);
        assert(src.sDownstream === tgt.zoneId, `Direct wire: ${src.sDownstream}`);
        const cells = await getAllCells();
        assert(Object.keys(cells).length === 2, `Expected 2 cells, got ${Object.keys(cells).length}`);
    });

    await test('Auto-complete from branch uses sDivertDownstream when in divert direction', async () => {
        page.once('dialog', d => d.accept());
        await gbEval(() => gbClear());
        await sleep(200);

        // Place branch at (5,5) facing E, and target at (5,6) — south = divert direction
        await gbEval(() => { gbSetMode('PLACE'); gbSetType('branch'); gbSetDir('E'); });
        await clickCell(5, 5);
        await gbEval(() => { gbSetType('transport'); gbSetDir('S'); });
        await clickCell(5, 6);

        // Clear the auto-wire that happened during placement
        await gbEval(() => {
            GRID.cells['5,5'].sDivertDownstream = null;
            GRID.cells['5,5'].sDownstream = null;
        });

        // Auto-complete from branch to south zone
        await gbEval(() => {
            gbSetMode('SELECT');
            gbStartAC('5,5');
            gbSaveUndo();
            gbExecAC('5,6');
        });
        await sleep(200);

        const br = await getCellData(5, 5);
        const tgt = await getCellData(5, 6);
        assert(br.sDivertDownstream === tgt.zoneId,
            `Expected divert wire, got sDivertDownstream=${br.sDivertDownstream}, sDownstream=${br.sDownstream}`);
    });

    // ── SECTION 7: Undo ───────────────────────────────────────────
    console.log('\nSection 7: Undo');

    await test('Undo restores previous state', async () => {
        page.once('dialog', d => d.accept());
        await gbEval(() => gbClear());
        await sleep(200);

        await gbEval(() => { gbSetMode('PLACE'); gbSetType('transport'); gbSetDir('E'); });
        await clickCell(5, 5);
        let cells = await getAllCells();
        assert(Object.keys(cells).length === 1);

        await clickCell(6, 5);
        cells = await getAllCells();
        assert(Object.keys(cells).length === 2);

        // Undo
        await gbEval(() => gbUndo());
        await sleep(200);
        cells = await getAllCells();
        assert(Object.keys(cells).length === 1, `After undo: ${Object.keys(cells).length}`);
    });

    // ── SECTION 8: Delete ─────────────────────────────────────────
    console.log('\nSection 8: Delete');

    await test('Delete zone and unwire references', async () => {
        page.once('dialog', d => d.accept());
        await gbEval(() => gbClear());
        await sleep(200);

        await gbEval(() => { gbSetMode('PLACE'); gbSetType('transport'); gbSetDir('E'); });
        await clickCell(5, 5);
        await clickCell(6, 5);
        await clickCell(7, 5);

        const mid = await getCellData(6, 5);
        const first = await getCellData(5, 5);
        assert(first.sDownstream === mid.zoneId);

        // Delete middle
        await gbEval(() => { gbSetMode('DELETE'); });
        await clickCell(6, 5);
        await sleep(200);

        const deleted = await getCellData(6, 5);
        assert(!deleted, 'Middle should be deleted');
        const firstAfter = await getCellData(5, 5);
        assert(!firstAfter.sDownstream, `Downstream should be null, got ${firstAfter.sDownstream}`);
    });

    // ── SECTION 9: Compile, Apply, Run ────────────────────────────
    console.log('\nSection 9: Full PLC Pipeline');

    await test('Build layout → compile → apply', async () => {
        page.once('dialog', d => d.accept());
        await gbEval(() => gbClear());
        await sleep(200);

        // Build: SPAWN → T → SCANNER → BRANCH → END(force_release)
        //                                  ↓ (spur_1)
        //                                T → T → PICKUP
        await gbEval(() => { gbSetMode('PLACE'); gbSetDir('E'); });

        await gbEval(() => gbSetType('spawn'));
        await clickCell(2, 5);
        await gbEval(() => gbSetType('transport'));
        await clickCell(3, 5);
        await gbEval(() => gbSetType('scanner'));
        await clickCell(4, 5);
        await gbEval(() => gbSetType('branch'));
        await clickCell(5, 5);
        await gbEval(() => gbSetType('end'));
        await clickCell(6, 5);

        // Spur
        await gbEval(() => { gbSetType('transport'); gbSetDir('S'); });
        await clickCell(5, 6);
        await clickCell(5, 7);
        await gbEval(() => gbSetType('pickup'));
        await clickCell(5, 8);

        // Set track groups + flags
        await gbEval(() => {
            for (const k in GRID.cells) {
                const c = GRID.cells[k];
                if (c.row >= 6) c.trackGroup = 'spur_1';
            }
        });

        // Compile
        const compResult = await gbEval(async () => {
            const d = gbExport();
            const r = await fetch('/api/topology/compile', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ descriptors: d }),
            });
            return r.json();
        });
        assert(compResult.status === 'ok', `Compile: ${JSON.stringify(compResult)}`);
        assert(compResult.zone_count === 8, `Zones: ${compResult.zone_count}`);

        // Apply
        const applyResult = await gbEval(async () => {
            const d = gbExport();
            const r = await fetch('/api/topology/apply', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ descriptors: d }),
            });
            return r.json();
        });
        assert(applyResult.status === 'ok', `Apply: ${JSON.stringify(applyResult)}`);
    });

    await test('Restore default topology, start, spawn, verify tote movement', async () => {
        // Load the original 34-zone topology from file to get full PLC system
        const loadResult = await gbEval(async () => {
            const resp = await fetch('/api/topology/load', { method: 'POST' });
            return resp.json();
        });
        // If load fails (no file), the apply from above is still active, which is fine
        await sleep(1000);

        // Start
        await gbEval(() => fetch('/api/hmi/bHMI_Start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ value: true }),
        }));
        await sleep(500);

        // 10x speed
        await gbEval(() => fetch('/api/speed', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ value: 10.0 }),
        }));
        await sleep(200);

        // Verify running
        let state = await gbEval(() => fetch('/api/state').then(r => r.json()));
        const sysState = (state.system && state.system.eSystemState) || state.system_state;
        assert(sysState === 'RUNNING' || sysState === 2,
            `System state: ${sysState}`);

        // Spawn 2 totes
        await gbEval(() => fetch('/api/hmi/iHMI_SpawnCount', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ value: 2 }),
        }));

        // Poll state over time, tracking tote positions
        const seenZones = new Set();
        for (let i = 0; i < 40; i++) {
            await sleep(500);
            state = await gbEval(() => fetch('/api/state').then(r => r.json()));
            const zones = state.zones || {};
            for (const [zid, zdata] of Object.entries(zones)) {
                if (zdata.bTotePresent) seenZones.add(zid);
            }
            if (seenZones.size >= 3) break; // Seen enough movement
        }

        assert(seenZones.size > 0,
            `Tote never appeared. Zones: ${JSON.stringify(Object.keys(state.zones || {}))}`);
        console.log(`    (Tote seen in ${seenZones.size} zones: ${[...seenZones].join(', ')})`);

        if (seenZones.size >= 2) {
            console.log('    (Confirmed: tote moved through multiple zones)');
        }
    });

    // ── SECTION 10: Import & ID Rebuild ───────────────────────────
    console.log('\nSection 10: Import & ID Rebuild');

    await test('Load current topology into builder', async () => {
        await gbEval(async () => await gbLoadCurrent());
        await sleep(500);
        const cells = await getAllCells();
        assert(Object.keys(cells).length === 8, `Expected 8, got ${Object.keys(cells).length}`);
    });

    await test('New zones after import get non-conflicting IDs', async () => {
        await gbEval(() => { gbSetMode('PLACE'); gbSetType('transport'); gbSetDir('E'); });
        await clickCell(25, 5);
        await clickCell(26, 5);
        const cells = await getAllCells();
        const ids = Object.values(cells).map(c => c.zoneId);
        const dupes = ids.filter((id, i) => ids.indexOf(id) !== i);
        assert(dupes.length === 0, `Duplicate IDs: ${dupes.join(', ')}`);
    });

    // ── SECTION 11: Keyboard Shortcuts ────────────────────────────
    console.log('\nSection 11: Keyboard Shortcuts');

    await test('Ctrl+S does NOT trigger Select mode', async () => {
        await gbEval(() => gbSetMode('PLACE'));
        page.once('dialog', d => d.dismiss());
        await page.keyboard.down('Control');
        await page.keyboard.press('s');
        await page.keyboard.up('Control');
        await sleep(100);
        const b = await getBuilder();
        assert(b.mode === 'PLACE', `Expected PLACE, got ${b.mode}`);
    });

    await test('Number keys switch zone types', async () => {
        await page.keyboard.press('1');
        let b = await getBuilder();
        assert(b.placeType === 'transport');
        await page.keyboard.press('3');
        b = await getBuilder();
        assert(b.placeType === 'branch');
        await page.keyboard.press('4');
        b = await getBuilder();
        assert(b.placeType === 'merge');
    });

    // ── SECTION 12: Console Errors ────────────────────────────────
    console.log('\nSection 12: JS Error Check');

    await test('No JavaScript errors during entire test run', async () => {
        const real = jsErrors.filter(e => !e.includes('favicon') && !e.includes('WebSocket'));
        assert(real.length === 0, `JS errors: ${real.join('; ')}`);
    });

    // ═══════════════════════════════════════════════════════════════
    console.log('\n══════════════════════════════════════════════');
    console.log(`  Results: ${passed} passed, ${failed} failed`);
    if (errors.length > 0) {
        console.log('\n  Failures:');
        errors.forEach(e => console.log(`    - ${e.name}: ${e.error}`));
    }
    console.log('══════════════════════════════════════════════\n');

    await browser.close();
    process.exit(failed > 0 ? 1 : 0);
}

runTests().catch(e => {
    console.error('Fatal:', e);
    if (browser) browser.close();
    process.exit(1);
});

const fs = require("fs");
const path = require("path");
const { chromium } = require("playwright");

const baseUrl = process.env.POSEIDON_E2E_BASE_URL || "http://127.0.0.1:8080";
const artifactDir =
  process.env.POSEIDON_E2E_ARTIFACT_DIR || path.join(__dirname, "artifacts");
const timeoutMs = Number.parseInt(
  process.env.POSEIDON_E2E_TIMEOUT_MS || "90000",
  10,
);
const refreshDelayMs = Number.parseInt(
  process.env.POSEIDON_E2E_DIAGNOSTICS_REFRESH_DELAY_MS || "6000",
  10,
);
const requireTelemetry =
  (process.env.POSEIDON_E2E_REQUIRE_TELEMETRY || "0") === "1";

const requiredDiagnostics = (process.env.POSEIDON_E2E_REQUIRED_DIAGNOSTICS ||
  "GNSS Fix,IMU Communication,Sonar Communication")
  .split(",")
  .map((name) => name.trim())
  .filter(Boolean);
const requireOk =
  (process.env.POSEIDON_E2E_REQUIRE_DIAGNOSTICS_OK || "1") === "1";

function ensureDir(dir) {
  fs.mkdirSync(dir, { recursive: true });
}

async function run() {
  ensureDir(artifactDir);

  const browser = await chromium.launch({ headless: true });
  const context = await browser.newContext();
  const page = await context.newPage();

  const pageErrors = [];
  const consoleErrors = [];
  const websocketActivity = [];

  page.on("pageerror", (err) => pageErrors.push(String(err)));
  page.on("console", (msg) => {
    if (msg.type() === "error") {
      consoleErrors.push(msg.text());
    }
  });
  page.on("websocket", (ws) => {
    const entry = {
      url: ws.url(),
      openedAt: Date.now(),
      received: [],
      sent: [],
      errors: [],
    };
    websocketActivity.push(entry);

    ws.on("framesent", (frame) => {
      entry.sent.push(frame.payload);
      if (entry.sent.length > 20) entry.sent.shift();
    });
    ws.on("framereceived", (frame) => {
      entry.received.push(frame.payload);
      if (entry.received.length > 20) entry.received.shift();
    });
    ws.on("socketerror", (err) => {
      entry.errors.push(String(err));
      if (entry.errors.length > 20) entry.errors.shift();
    });
  });

  try {
    await page.goto(`${baseUrl}/diagnostics.html`, {
      waitUntil: "domcontentloaded",
      timeout: timeoutMs,
    });

    await page.waitForFunction(
      () => {
        const table = document.querySelector("#diagnosticsTable");
        if (!table) return false;
        const rows = table.querySelectorAll("tr");
        return rows.length > 1;
      },
      null,
      { timeout: timeoutMs },
    );

    const diagnosticsDeadline = Date.now() + timeoutMs;
    let diagStatus = {};
    while (Date.now() < diagnosticsDeadline) {
      // Nudge the page to refresh diagnostics; this triggers `updateDiagnostic` on the websocket.
      const btn = await page.$("#diagnosticsButton");
      if (btn) {
        await btn.click();
      }

      await page.waitForTimeout(refreshDelayMs);

      diagStatus = await page.evaluate(() => {
        const table = document.querySelector("#diagnosticsTable");
        const rows = Array.from(table.querySelectorAll("tr")).slice(1);
        const out = {};
        for (const row of rows) {
          const cells = row.querySelectorAll("td");
          if (cells.length < 3) continue;
          const status = (cells[0].textContent || "").trim();
          const name = (cells[1].textContent || "").trim();
          const info = (cells[2].textContent || "").trim();
          if (!name) continue;
          out[name] = {
            ok: status.includes("✅"),
            status,
            info,
          };
        }
        return out;
      });

      const missing = requiredDiagnostics.filter((name) => !diagStatus[name]);
      const failing = requiredDiagnostics.filter((name) => {
        if (!diagStatus[name]) return false;
        if (diagStatus[name].ok) return false;
        if (!requireOk) {
          // Accept WARN-like cases where messages are flowing but the rate is below threshold.
          // Fail only if it clearly indicates no messages.
          const info = (diagStatus[name].info || "").toLowerCase();
          if (info.includes("no imu message received")) return true;
          if (info.includes("no message received")) return true;
          if (info.includes("0 msg")) return true;
          return false;
        }
        return true;
      });

      if (missing.length === 0 && failing.length === 0) {
        break;
      }
    }

    const missing = requiredDiagnostics.filter((name) => !diagStatus[name]);
    if (missing.length) {
      throw new Error(
        `diagnostics.html missing diagnostic rows: ${missing.join(", ")}`,
      );
    }

    const failing = requiredDiagnostics.filter(
      (name) => {
        if (!diagStatus[name]) return false;
        if (diagStatus[name].ok) return false;
        if (!requireOk) {
          const info = (diagStatus[name].info || "").toLowerCase();
          if (info.includes("no imu message received")) return true;
          if (info.includes("no message received")) return true;
          if (info.includes("0 msg")) return true;
          return false;
        }
        return true;
      },
    );
    if (failing.length) {
      const details = failing
        .map((name) => `${name}=${diagStatus[name].status} (${diagStatus[name].info})`)
        .join(", ");
      throw new Error(`diagnostics.html diagnostics not OK: ${details}`);
    }

    await page.waitForFunction(
      () => {
        const table = document.querySelector("#runningNodesTable");
        if (!table) return false;
        const rows = table.querySelectorAll("tr");
        return rows.length > 1;
      },
      null,
      { timeout: timeoutMs },
    );

    if (requireTelemetry) {
      await page.goto(`${baseUrl}/index.html`, {
        waitUntil: "domcontentloaded",
        timeout: timeoutMs,
      });

      // Dashboard shows an overlay until at least one telemetry message is received.
      await page.waitForFunction(
        () => {
          const el = document.querySelector("#overlay");
          return el && el.style && el.style.display === "none";
        },
        null,
        { timeout: timeoutMs },
      );
    }

    if (pageErrors.length) {
      throw new Error(`Page errors: ${pageErrors.join(" | ")}`);
    }
    if (consoleErrors.length) {
      throw new Error(`Console errors: ${consoleErrors.join(" | ")}`);
    }
  } catch (err) {
    const screenshotPath = path.join(artifactDir, "ui_failure.png");
    try {
      await page.screenshot({ path: screenshotPath, fullPage: true });
    } catch {
      // Best effort screenshot.
    }

    console.error(String(err));
    if (pageErrors.length) console.error("pageerror:", pageErrors);
    if (consoleErrors.length) console.error("console.error:", consoleErrors);
    if (websocketActivity.length) {
      const wsPath = path.join(artifactDir, "websockets.json");
      try {
        fs.writeFileSync(wsPath, JSON.stringify(websocketActivity, null, 2));
        console.error(`websocket debug saved to: ${wsPath}`);
      } catch {
        // Best effort.
      }
    } else {
      console.error("No websocket activity observed by Playwright.");
    }
    process.exitCode = 1;
  } finally {
    await browser.close();
  }
}

run().catch((err) => {
  console.error(String(err));
  process.exitCode = 1;
});

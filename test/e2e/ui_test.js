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

const requiredDiagnostics = (process.env.POSEIDON_E2E_REQUIRED_DIAGNOSTICS ||
  "GNSS Fix,IMU Communication,Sonar Communication")
  .split(",")
  .map((name) => name.trim())
  .filter(Boolean);

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

  page.on("pageerror", (err) => pageErrors.push(String(err)));
  page.on("console", (msg) => {
    if (msg.type() === "error") {
      consoleErrors.push(msg.text());
    }
  });

  try {
    await page.goto(`${baseUrl}/status.html`, {
      waitUntil: "domcontentloaded",
      timeout: timeoutMs,
    });

    await page.waitForFunction(() => {
      const el = document.querySelector("#uptimeText");
      return el && el.textContent && el.textContent.trim() !== "N/A";
    }, { timeout: timeoutMs });

    const uptimeText = (await page.textContent("#uptimeText"))?.trim();
    if (!uptimeText || uptimeText === "N/A") {
      throw new Error(`status.html did not populate uptime (got: ${uptimeText})`);
    }

    await page.goto(`${baseUrl}/diagnostics.html`, {
      waitUntil: "domcontentloaded",
      timeout: timeoutMs,
    });

    await page.waitForFunction(() => {
      const table = document.querySelector("#diagnosticsTable");
      if (!table) return false;
      const rows = table.querySelectorAll("tr");
      return rows.length > 1;
    }, { timeout: timeoutMs });

    const tableText = (await page.textContent("#diagnosticsTable")) || "";
    for (const name of requiredDiagnostics) {
      if (!tableText.includes(name)) {
        throw new Error(`diagnostics.html missing diagnostic row: ${name}`);
      }
    }

    await page.waitForFunction(() => {
      const table = document.querySelector("#runningNodesTable");
      if (!table) return false;
      const rows = table.querySelectorAll("tr");
      return rows.length > 1;
    }, { timeout: timeoutMs });

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
    process.exitCode = 1;
  } finally {
    await browser.close();
  }
}

run().catch((err) => {
  console.error(String(err));
  process.exitCode = 1;
});


#!/usr/bin/env node

import fs from "fs";
import os from "os";
import path from "path";
import { spawn } from "child_process";
import { fileURLToPath } from "url";
import { chromium } from "playwright";

const SCRIPT_DIR = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(SCRIPT_DIR, "..", "..");
const PROFILE_DIR = path.join(REPO_ROOT, "docker", "home", ".box-profile");
const DOWNLOAD_DIR = path.join(os.homedir(), "Downloads");
const DIAGNOSTIC_DIR = path.join(PROFILE_DIR, "diagnostics");
const GITHUB_LOGIN_MARKER = path.join(PROFILE_DIR, ".github-login-complete");
const BOX_LOGIN_MARKER = path.join(PROFILE_DIR, ".box-login-complete");
const DOWNLOAD_HELPER = path.join(SCRIPT_DIR, "download-helper.sh");
const BOX_HOST_PATTERN = /box\.com/i;
const GITHUB_HOST_PATTERN = /github\.com/i;
const CHROMIUM_PROFILE_LOCK_FILES = ["SingletonLock", "SingletonCookie", "SingletonSocket"];
const BOX_FOLDER_SETTLE_MS = 3000;
const LOGIN_WAIT_TIMEOUT_MS = 10 * 60 * 1000;
const LOGIN_POLL_INTERVAL_MS = 2000;
const LOGIN_STATUS_INTERVAL_MS = 15000;
const ISSUE_PAGE_MAX_ATTEMPTS = 3;
const ISSUE_PAGE_RETRY_WAIT_MS = 2000;
const ISSUE_PAGE_SCAN_TIMEOUT_MS = 15000;
const ISSUE_PAGE_SCAN_POLL_INTERVAL_MS = 1500;
const ISSUE_PAGE_STATUS_INTERVAL_MS = 5000;
const BOX_FOLDER_MAX_ATTEMPTS = 4;
const BOX_FOLDER_RETRY_WAIT_MS = 3000;
const BOX_FOLDER_SCAN_TIMEOUT_MS = 25000;
const BOX_FOLDER_SCAN_POLL_INTERVAL_MS = 1500;
const BOX_FOLDER_STATUS_INTERVAL_MS = 7000;
const BOX_LIST_VISIBLE_TIMEOUT_MS = 8000;
const BOX_LIST_VISIBLE_POLL_INTERVAL_MS = 1000;
const DIAGNOSTIC_TEXT_LIMIT = 500;
const NAVIGATION_MAX_ATTEMPTS = 3;
const NAVIGATION_RETRY_WAIT_MS = 3000;
const MAIN_FLOW_MAX_ATTEMPTS = 3;
const MAIN_FLOW_RETRY_WAIT_MS = 5000;
const CONSOLE_REPEAT_LIMIT = 3;
const BOX_DOWNLOAD_CONCURRENCY = 3;
const BOX_FILE_DOWNLOAD_MAX_ATTEMPTS = 3;
const BOX_FILE_DOWNLOAD_RETRY_WAIT_MS = 5000;

const PAGE_IDS = new WeakMap();
const DIAGNOSTIC_ATTACHED = new WeakSet();
const CONSOLE_MESSAGE_COUNTS = new Map();
let NEXT_PAGE_ID = 1;

function ensureDir(dirPath) {
  fs.mkdirSync(dirPath, { recursive: true });
}

function getPageId(page) {
  const existing = PAGE_IDS.get(page);
  if (existing) {
    return existing;
  }
  const assigned = NEXT_PAGE_ID;
  NEXT_PAGE_ID += 1;
  PAGE_IDS.set(page, assigned);
  return assigned;
}

function clipText(value, limit = DIAGNOSTIC_TEXT_LIMIT) {
  const text = String(value ?? "").replace(/\s+/g, " ").trim();
  if (text.length <= limit) {
    return text;
  }
  return `${text.slice(0, limit)}...`;
}

function isTargetHost(url) {
  if (!url) {
    return false;
  }
  return BOX_HOST_PATTERN.test(url) || GITHUB_HOST_PATTERN.test(url);
}

function logBrowserDiagnostic(page, category, message) {
  const pageId = getPageId(page);
  const pageUrl = clipText(page.url() || "(no-url)", 160);
  console.log(`[browser:${category}][p${pageId}] ${message} (page=${pageUrl})`);
}

function shouldLogConsoleMessage(msg) {
  const type = msg.type();
  const text = clipText(msg.text(), 240);
  if (type === "error") {
    return true;
  }
  if (type === "warning") {
    return /insufficient permissions|failed to load resource|err_/i.test(text);
  }
  return false;
}

function shouldEmitMessageOnce(kind, message) {
  const key = `${kind}:${message}`;
  const count = CONSOLE_MESSAGE_COUNTS.get(key) || 0;
  if (count >= CONSOLE_REPEAT_LIMIT) {
    return false;
  }
  CONSOLE_MESSAGE_COUNTS.set(key, count + 1);
  return true;
}

function isRetriableNavigationError(error) {
  const message = String(error?.message || error);
  return message.includes("net::")
    || message.includes("Timeout")
    || message.includes("Target page, context or browser has been closed");
}

async function gotoWithRetry(page, url, label, options = {}) {
  const {
    waitUntil = "domcontentloaded",
    maxAttempts = NAVIGATION_MAX_ATTEMPTS,
    retryWaitMs = NAVIGATION_RETRY_WAIT_MS,
  } = options;

  for (let attempt = 1; attempt <= maxAttempts; attempt++) {
    try {
      await page.goto(url, { waitUntil });
      return;
    } catch (error) {
      if (!isRetriableNavigationError(error) || attempt === maxAttempts) {
        throw error;
      }
      console.log(
        `Navigation failed for ${label} (attempt ${attempt}/${maxAttempts}); retrying in ${Math.ceil(retryWaitMs / 1000)}s...`,
      );
      await page.waitForTimeout(retryWaitMs);
    }
  }
}

function attachPageDiagnostics(page) {
  if (DIAGNOSTIC_ATTACHED.has(page)) {
    return;
  }
  DIAGNOSTIC_ATTACHED.add(page);

  const pageId = getPageId(page);
  console.log(`[browser:page][p${pageId}] attached diagnostics`);

  page.on("console", (msg) => {
    const location = msg.location();
    const locationUrl = location?.url || "";
    const onTarget = isTargetHost(page.url()) || isTargetHost(locationUrl);
    if (!onTarget) {
      return;
    }
    if (!shouldLogConsoleMessage(msg)) {
      return;
    }
    const locationInfo = locationUrl
      ? ` @${clipText(locationUrl, 120)}:${(location.lineNumber ?? 0) + 1}:${(location.columnNumber ?? 0) + 1}`
      : "";
    const message = `${clipText(msg.text())}${locationInfo}`;
    if (!shouldEmitMessageOnce(`console:${msg.type()}`, message)) {
      return;
    }
    logBrowserDiagnostic(page, `console:${msg.type()}`, message);
  });

  page.on("pageerror", (error) => {
    if (!isTargetHost(page.url())) {
      return;
    }
    logBrowserDiagnostic(page, "pageerror", clipText(error?.message || error));
  });

  page.on("requestfailed", (request) => {
    const requestUrl = request.url();
    if (!isTargetHost(requestUrl)) {
      return;
    }
    const failure = request.failure();
    const reason = failure?.errorText || "unknown";
    logBrowserDiagnostic(
      page,
      "requestfailed",
      `${request.method()} ${clipText(requestUrl, 180)} reason=${clipText(reason, 180)}`,
    );
  });

  page.on("response", (response) => {
    const responseUrl = response.url();
    if (!isTargetHost(responseUrl)) {
      return;
    }
    const status = response.status();
    if (status < 400) {
      return;
    }
    logBrowserDiagnostic(
      page,
      "response",
      `status=${status} ${clipText(response.request().method(), 16)} ${clipText(responseUrl, 180)}`,
    );
  });
}

function setupContextDiagnostics(context) {
  for (const page of context.pages()) {
    attachPageDiagnostics(page);
  }
  context.on("page", (page) => {
    attachPageDiagnostics(page);
  });
}

function makeDiagnosticFileTag(label) {
  return String(label || "state")
    .toLowerCase()
    .replace(/[^a-z0-9._-]+/g, "-")
    .replace(/^-+|-+$/g, "")
    .slice(0, 80) || "state";
}

function utcTimestampTag() {
  return new Date().toISOString().replace(/[:.]/g, "-");
}

async function capturePageState(page, label) {
  if (!page) {
    return;
  }

  try {
    ensureDir(DIAGNOSTIC_DIR);
    const pageId = getPageId(page);
    const tag = makeDiagnosticFileTag(label);
    const stamp = utcTimestampTag();
    const screenshotPath = path.join(DIAGNOSTIC_DIR, `${stamp}-p${pageId}-${tag}.png`);
    const title = await page.title().catch(() => "(title-unavailable)");
    const url = page.url() || "(url-unavailable)";

    await page.screenshot({ path: screenshotPath, fullPage: true }).catch(async () => {
      await page.screenshot({ path: screenshotPath, fullPage: false }).catch(() => {});
    });

    console.error(`[diagnostic] ${label} url=${url}`);
    console.error(`[diagnostic] ${label} title=${clipText(title, 180)}`);
    console.error(`[diagnostic] ${label} screenshot=${screenshotPath}`);
  } catch (error) {
    console.error(`[diagnostic] failed to capture page state (${label}): ${error?.message || error}`);
  }
}

function clearChromiumProfileLocks() {
  const removed = [];
  for (const name of CHROMIUM_PROFILE_LOCK_FILES) {
    const target = path.join(PROFILE_DIR, name);
    try {
      // lstat handles broken symlinks; existsSync does not.
      fs.lstatSync(target);
      fs.rmSync(target, { force: true });
      removed.push(name);
    } catch (error) {
      if (error?.code !== "ENOENT") {
        throw error;
      }
    }
  }

  if (removed.length > 0) {
    console.log(`Removed stale Chromium profile locks: ${removed.join(", ")}`);
  }
}

async function launchPersistentContextWithRetry() {
  clearChromiumProfileLocks();
  try {
    return await chromium.launchPersistentContext(PROFILE_DIR, {
      headless: false,
      acceptDownloads: true,
      downloadsPath: DOWNLOAD_DIR,
      viewport: { width: 1440, height: 960 },
    });
  } catch (error) {
    const message = String(error?.message || error);
    if (!message.includes("profile appears to be in use")) {
      throw error;
    }

    clearChromiumProfileLocks();
    return chromium.launchPersistentContext(PROFILE_DIR, {
      headless: false,
      acceptDownloads: true,
      downloadsPath: DOWNLOAD_DIR,
      viewport: { width: 1440, height: 960 },
    });
  }
}

function cleanupStaleDownloads(prefix) {
  const staleFiles = fs.readdirSync(DOWNLOAD_DIR).filter((name) => {
    return name === `${prefix}_log.tar`
      || name === `${prefix}_ros2_topics.tar`
      || name.startsWith(`${prefix}_ros2_topics_part_`);
  });

  for (const name of staleFiles) {
    fs.rmSync(path.join(DOWNLOAD_DIR, name), { force: true });
  }

  if (staleFiles.length > 0) {
    console.log(`Removed stale downloads: ${staleFiles.join(", ")}`);
  }
}

function getIssueInfo(issueUrl) {
  const url = new URL(issueUrl);
  const match = url.pathname.match(/^\/([^/]+)\/([^/]+)\/issues\/(\d+)/);
  if (!match) {
    throw new Error(`Unsupported GitHub issue URL: ${issueUrl}`);
  }

  const [, owner, repo, issueNumber] = match;
  return {
    owner,
    repo,
    issueNumber,
    issueTag: `${owner}-${issueNumber}`,
  };
}

function normalizeHref(href, baseUrl) {
  try {
    return new URL(href, baseUrl).toString();
  } catch {
    return null;
  }
}

async function waitForStableDom(page, timeoutMs = 15000) {
  await page.waitForLoadState("domcontentloaded");
  await page.waitForTimeout(1500);
  await page.locator("body").first().waitFor({ state: "visible", timeout: timeoutMs });
}

function getPrefixAndPartCount(boxLinks) {
  const fileEntries = boxLinks.filter((item) => item.name && item.name !== item.href);
  const ros2Parts = fileEntries
    .map((item) => item.name.match(/^(.*)_ros2_topics_part_[a-z]{2}$/i))
    .filter(Boolean);
  const logTar = fileEntries.find((item) => /_log\.tar$/i.test(item.name));

  if (!logTar || ros2Parts.length === 0) {
    throw new Error("Could not find *_log.tar and *_ros2_topics_part_* links in the first comment.");
  }

  const prefix = logTar.name.replace(/_log\.tar$/i, "");
  const partCount = ros2Parts.filter((match) => match[1] === prefix).length;
  if (partCount === 0) {
    throw new Error(`Could not determine ros2 topic part count for prefix ${prefix}.`);
  }

  return { prefix, partCount };
}

function getFolderAndTargetNames(boxLinks) {
  const folderLink = boxLinks.find((item) => /\/folder\//i.test(item.href));
  const targetFiles = boxLinks.filter((item) => {
    return item.name && !/\/folder\//i.test(item.href) && /^cabot_/i.test(item.name);
  });

  return {
    folderLink,
    targetNames: targetFiles.map((item) => item.name),
  };
}

async function collectIssueBoxLinks(page, issueUrl) {
  const commentLocators = [
    page.locator(".js-timeline-item .comment-body"),
    page.locator(".timeline-comment .comment-body"),
    page.locator("[data-testid='issue-body']"),
    page.locator(".edit-comment-hide .comment-body"),
    page.locator(".comment-body"),
  ];

  for (const locator of commentLocators) {
    const count = await locator.count();
    for (let i = 0; i < count; i++) {
      const comment = locator.nth(i);
      const links = comment.locator("a[href*='box.com']");
      const linkCount = await links.count();
      if (linkCount === 0) {
        continue;
      }

      const items = [];
      for (let j = 0; j < linkCount; j++) {
        const link = links.nth(j);
        const href = await link.getAttribute("href");
        const name = (await link.innerText().catch(() => "")).trim();
        const resolvedHref = href ? normalizeHref(href, issueUrl) : null;
        if (!resolvedHref || !BOX_HOST_PATTERN.test(resolvedHref)) {
          continue;
        }
        items.push({ name, href: resolvedHref });
      }

      if (items.length > 0) {
        return items;
      }
    }
  }

  return [];
}

async function extractFirstCommentBoxLinks(page, issueUrl, options = {}) {
  const { navigate = true, maxAttempts = ISSUE_PAGE_MAX_ATTEMPTS } = options;

  for (let attempt = 1; attempt <= maxAttempts; attempt++) {
    if (navigate || attempt > 1) {
      await gotoWithRetry(page, issueUrl, "issue page");
    }
    await waitForStableDom(page);

    const scanDeadline = Date.now() + ISSUE_PAGE_SCAN_TIMEOUT_MS;
    let nextStatus = Date.now() + ISSUE_PAGE_STATUS_INTERVAL_MS;

    while (Date.now() < scanDeadline) {
      const items = await collectIssueBoxLinks(page, issueUrl);
      if (items.length > 0) {
        return items;
      }

      if (Date.now() >= nextStatus) {
        const remainingSec = Math.max(0, Math.ceil((scanDeadline - Date.now()) / 1000));
        console.log(`Waiting for Box links on issue page... (${remainingSec}s before retry)`);
        nextStatus = Date.now() + ISSUE_PAGE_STATUS_INTERVAL_MS;
      }

      await page.waitForTimeout(ISSUE_PAGE_SCAN_POLL_INTERVAL_MS);
    }

    if (attempt < maxAttempts) {
      console.log(`No Box links found on issue page (attempt ${attempt}/${maxAttempts}); reloading...`);
      await page.waitForTimeout(ISSUE_PAGE_RETRY_WAIT_MS);
    }
  }

  throw new Error(`No Box links found in the first issue comment/body after ${maxAttempts} attempts.`);
}

async function clickFirstVisible(page, selectors, timeoutMs = 2000) {
  for (const selector of selectors) {
    const locator = page.locator(selector).first();
    if (await locator.isVisible({ timeout: timeoutMs }).catch(() => false)) {
      await locator.click();
      return true;
    }
  }
  return false;
}

async function pageNeedsGitHubLogin(page) {
  if (/github\.com\/login/i.test(page.url())) {
    return true;
  }

  const loginSelectors = [
    "input[name='login']",
    "input[name='password']",
    "button:has-text('Sign in')",
    "a:has-text('Sign in')",
  ];

  for (const selector of loginSelectors) {
    if (await page.locator(selector).first().isVisible({ timeout: 1000 }).catch(() => false)) {
      return true;
    }
  }

  return false;
}

async function pageNeedsBoxLogin(page) {
  if (/box\.com\/login/i.test(page.url()) || /account\.box\.com\/login/i.test(page.url())) {
    return true;
  }

  const loginSelectors = [
    "form input[type='email']",
    "form input[name='login']",
    "form input[type='password']",
    "form input[name='password']",
  ];

  for (const selector of loginSelectors) {
    if (await page.locator(selector).first().isVisible({ timeout: 1000 }).catch(() => false)) {
      return true;
    }
  }

  return false;
}

async function waitForLoginCompletion(page, serviceName, promptUrl, needsLogin) {
  console.log(`${serviceName} login is required. Waiting for completion...`);
  if (promptUrl) {
    console.log(`Target page: ${promptUrl}`);
  }

  const deadline = Date.now() + LOGIN_WAIT_TIMEOUT_MS;
  let nextStatus = Date.now() + LOGIN_STATUS_INTERVAL_MS;

  while (Date.now() < deadline) {
    await page.waitForTimeout(LOGIN_POLL_INTERVAL_MS);
    const mustLogin = await needsLogin(page).catch(() => true);
    if (!mustLogin) {
      console.log(`${serviceName} login completed.`);
      return;
    }

    if (Date.now() >= nextStatus) {
      const remainingSec = Math.max(0, Math.ceil((deadline - Date.now()) / 1000));
      console.log(`Still waiting for ${serviceName} login... (${remainingSec}s left)`);
      nextStatus = Date.now() + LOGIN_STATUS_INTERVAL_MS;
    }
  }

  throw new Error(`${serviceName} login did not complete within ${Math.floor(LOGIN_WAIT_TIMEOUT_MS / 1000)} seconds.`);
}

async function maybeCompleteLogin(page, markerPath, serviceName, promptUrl, needsLogin) {
  const firstRun = !fs.existsSync(markerPath);
  await waitForStableDom(page);
  const mustLogin = await needsLogin(page);

  if (!mustLogin) {
    if (firstRun) {
      fs.writeFileSync(markerPath, `${new Date().toISOString()}\n`);
    }
    console.log(`${serviceName} login is not required on this page.`);
    return;
  }

  if (!firstRun) {
    console.log(`Saved ${serviceName} session exists, but login is currently required.`);
  } else {
    console.log(`First ${serviceName} login detected.`);
  }

  await waitForLoginCompletion(page, serviceName, promptUrl, needsLogin);
  fs.writeFileSync(markerPath, `${new Date().toISOString()}\n`);
  await page.waitForTimeout(1000);
}

async function downloadSharedBoxFile(page, item) {
  for (let attempt = 1; attempt <= BOX_FILE_DOWNLOAD_MAX_ATTEMPTS; attempt++) {
    try {
      await gotoWithRetry(page, item.href, `download target ${item.name || item.href}`);
      await waitForStableDom(page);

      const downloadPromise = page.waitForEvent("download", { timeout: 120000 });
      const clicked = await clickFirstVisible(page, [
        "button:has-text('Download')",
        "a:has-text('Download')",
        "[aria-label*='Download']",
        "[data-testid*='download']",
        "button[data-resin-target='download']",
      ], 4000);

      if (!clicked) {
        throw new Error(`Download button not found for ${item.name || item.href}`);
      }

      const download = await downloadPromise;
      const filename = download.suggestedFilename();
      const preferredName = path.basename((item.name || "").trim() || filename);
      const destination = path.join(DOWNLOAD_DIR, preferredName);
      await download.saveAs(destination);
      console.log(`Downloaded: ${preferredName} (suggested: ${filename})`);
      return;
    } catch (error) {
      await capturePageState(page, `download-failure-${item.name || "unknown"}-attempt-${attempt}`);
      const message = String(error?.message || error);
      const retriable = message.includes("Timeout")
        || message.includes("Download button not found")
        || message.includes("net::")
        || /canceled/i.test(message)
        || message.includes("Target page, context or browser has been closed");
      if (!retriable || attempt === BOX_FILE_DOWNLOAD_MAX_ATTEMPTS) {
        throw error;
      }
      console.log(
        `Retrying download for ${item.name || item.href} `
        + `(attempt ${attempt}/${BOX_FILE_DOWNLOAD_MAX_ATTEMPTS}) in ${Math.ceil(BOX_FILE_DOWNLOAD_RETRY_WAIT_MS / 1000)}s...`,
      );
      await page.waitForTimeout(BOX_FILE_DOWNLOAD_RETRY_WAIT_MS);
    }
  }
}

async function downloadSharedBoxFiles(context, items, concurrency = BOX_DOWNLOAD_CONCURRENCY) {
  const queue = [...items];
  const workers = Array.from({ length: Math.min(concurrency, items.length) }, async () => {
    while (queue.length > 0) {
      const item = queue.shift();
      if (!item) {
        return;
      }

      const page = await context.newPage();
      try {
        await downloadSharedBoxFile(page, item);
      } finally {
        await page.close().catch(() => {});
      }
    }
  });

  await Promise.all(workers);
}

async function isBoxFolderListVisible(page) {
  const selectors = [
    "[role='grid']",
    "[role='treegrid']",
    "[role='rowgroup']",
    "[role='row']",
    "[data-testid*='item-list']",
    "[data-testid*='folder']",
  ];

  for (const selector of selectors) {
    const locator = page.locator(selector);
    const count = await locator.count().catch(() => 0);
    if (count > 0) {
      return true;
    }
  }

  return false;
}

async function waitForBoxFolderListVisible(page, timeoutMs = BOX_LIST_VISIBLE_TIMEOUT_MS) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    const visible = await isBoxFolderListVisible(page);
    if (visible) {
      return true;
    }
    await page.waitForTimeout(BOX_LIST_VISIBLE_POLL_INTERVAL_MS);
  }
  return false;
}

async function resolveFileLinksFromFolderOnce(page, folderUrl, targetNames) {
  const resolved = [];
  const missing = [];
  for (const targetName of targetNames) {
    const candidates = [
      page.locator(`a:has-text("${targetName}")`),
      page.locator("[role='row']").filter({ hasText: targetName }).locator("a"),
      page.locator("a").filter({ hasText: targetName }),
    ];

    let href = null;
    for (const locator of candidates) {
      const count = await locator.count().catch(() => 0);
      for (let i = 0; i < count; i++) {
        const item = locator.nth(i);
        const text = (await item.innerText().catch(() => "")).trim();
        if (!text.includes(targetName)) {
          continue;
        }
        const candidateHref = await item.getAttribute("href");
        if (!candidateHref) {
          continue;
        }
        href = normalizeHref(candidateHref, folderUrl);
        if (href) {
          break;
        }
      }
      if (href) {
        break;
      }
    }

    if (!href) {
      missing.push(targetName);
      continue;
    }

    resolved.push({ name: targetName, href });
  }

  return { resolved, missing };
}

async function resolveFileLinksFromFolder(page, folderUrl, targetNames, options = {}) {
  const { navigate = true } = options;

  if (navigate) {
    await gotoWithRetry(page, folderUrl, "Box folder page");
    await waitForStableDom(page);
  }

  // Box folder view can re-render right after initial paint; wait briefly before querying links.
  await page.waitForLoadState("networkidle", { timeout: 10000 }).catch(() => {});
  await page.waitForTimeout(BOX_FOLDER_SETTLE_MS);
  const listVisible = await waitForBoxFolderListVisible(page);
  if (!listVisible) {
    throw new Error("Box folder list is not visible.");
  }

  const scanDeadline = Date.now() + BOX_FOLDER_SCAN_TIMEOUT_MS;
  let nextStatus = Date.now() + BOX_FOLDER_STATUS_INTERVAL_MS;
  let latestMissing = [...targetNames];

  while (Date.now() < scanDeadline) {
    const { resolved, missing } = await resolveFileLinksFromFolderOnce(page, folderUrl, targetNames);
    if (missing.length === 0) {
      return resolved;
    }
    latestMissing = missing;

    if (Date.now() >= nextStatus) {
      const missingPreview = missing.slice(0, 3).join(", ");
      console.log(
        `Waiting for Box folder listing... found ${resolved.length}/${targetNames.length} files`
        + (missingPreview ? ` (missing: ${missingPreview}${missing.length > 3 ? ", ..." : ""})` : ""),
      );
      nextStatus = Date.now() + BOX_FOLDER_STATUS_INTERVAL_MS;
    }

    await page.waitForTimeout(BOX_FOLDER_SCAN_POLL_INTERVAL_MS);
  }

  throw new Error(
    `Could not find all target files in Box folder view: ${latestMissing.join(", ")}`,
  );
}

function startDownloadHelper(prefix, partCount, issueTag) {
  if (!fs.existsSync(DOWNLOAD_HELPER)) {
    throw new Error(`download-helper.sh not found: ${DOWNLOAD_HELPER}`);
  }

  const args = ["-p", prefix, "-s", "-n", String(partCount), "-i", issueTag];
  console.log(`Starting helper: ${DOWNLOAD_HELPER} ${args.join(" ")}`);

  const child = spawn(DOWNLOAD_HELPER, args, {
    cwd: DOWNLOAD_DIR,
    stdio: "inherit",
  });

  const helper = {
    child,
    stopping: false,
    completion: null,
  };

  const completion = new Promise((resolve, reject) => {
    child.on("error", reject);
    child.on("exit", (code, signal) => {
      if (code === 0) {
        resolve();
        return;
      }
      if (helper.stopping && signal === "SIGTERM") {
        resolve();
        return;
      }
      const reason = signal ? `signal ${signal}` : `code ${code}`;
      reject(new Error(`download-helper exited with ${reason}`));
    });
  });

  helper.completion = completion;
  return helper;
}

function stopDownloadHelper(helper) {
  if (!helper || !helper.child || helper.child.exitCode !== null) {
    return;
  }
  helper.stopping = true;
  helper.completion?.catch(() => {});
  helper.child.kill("SIGTERM");
}

async function main() {
  const issueUrl = process.argv[2];
  if (!issueUrl) {
    console.error("Usage: node script/box_download_logs.mjs <github-issue-url>");
    process.exit(1);
  }

  ensureDir(PROFILE_DIR);
  ensureDir(DOWNLOAD_DIR);
  ensureDir(DIAGNOSTIC_DIR);

  const issueInfo = getIssueInfo(issueUrl);
  const context = await launchPersistentContextWithRetry();
  setupContextDiagnostics(context);
  let helper = null;
  let page = null;
  let allowWholeFlowRetry = true;

  try {
    page = context.pages()[0] || await context.newPage();
    await gotoWithRetry(page, issueUrl, "issue page");
    await maybeCompleteLogin(page, GITHUB_LOGIN_MARKER, "GitHub", issueUrl, pageNeedsGitHubLogin);

    const boxLinks = await extractFirstCommentBoxLinks(page, issueUrl, { navigate: false });
    const filesToDownload = boxLinks.filter((item) => /\/s\/|\/file\/|\/folder\//i.test(item.href));
    const { prefix, partCount } = getPrefixAndPartCount(filesToDownload);
    const { folderLink, targetNames } = getFolderAndTargetNames(filesToDownload);

    console.log(`Issue: ${issueInfo.owner}/${issueInfo.repo}#${issueInfo.issueNumber}`);
    console.log(`Prefix: ${prefix}`);
    console.log(`Parts: ${partCount}`);

    if (!folderLink) {
      throw new Error("No Box folder link found in the first comment.");
    }
    if (targetNames.length === 0) {
      throw new Error("No target filenames found in the first comment.");
    }

    await gotoWithRetry(page, folderLink.href, "Box folder page");
    await maybeCompleteLogin(page, BOX_LOGIN_MARKER, "Box", folderLink.href, pageNeedsBoxLogin);

    let fileLinks = null;
    for (let attempt = 1; attempt <= BOX_FOLDER_MAX_ATTEMPTS; attempt++) {
      try {
        fileLinks = await resolveFileLinksFromFolder(page, folderLink.href, targetNames, { navigate: false });
        break;
      } catch (error) {
        const message = String(error?.message || error);
        const retriable = message.includes("Could not find")
          || message.includes("not visible");
        if (!retriable) {
          throw error;
        }
        if (attempt === BOX_FOLDER_MAX_ATTEMPTS) {
          throw error;
        }

        console.log(`Box folder is not ready (attempt ${attempt}/${BOX_FOLDER_MAX_ATTEMPTS}); reloading...`);
        await page.waitForTimeout(BOX_FOLDER_RETRY_WAIT_MS);
        await gotoWithRetry(page, folderLink.href, "Box folder page reload");
        await maybeCompleteLogin(page, BOX_LOGIN_MARKER, "Box", folderLink.href, pageNeedsBoxLogin);
      }
    }

    cleanupStaleDownloads(prefix);
    helper = startDownloadHelper(prefix, partCount, issueInfo.issueTag);
    allowWholeFlowRetry = false;
    await downloadSharedBoxFiles(context, fileLinks);
    await helper.completion;
    console.log(`Extracted under docker/home/sandbox/${issueInfo.issueTag}`);
  } catch (error) {
    if (!allowWholeFlowRetry && error && typeof error === "object") {
      error.noWholeFlowRetry = true;
    }
    await capturePageState(page, "main-flow-failure");
    throw error;
  } finally {
    stopDownloadHelper(helper);
    await context.close();
  }
}

async function runMainWithRetries() {
  for (let attempt = 1; attempt <= MAIN_FLOW_MAX_ATTEMPTS; attempt++) {
    try {
      await main();
      return;
    } catch (error) {
      if (error?.noWholeFlowRetry) {
        throw error;
      }
      if (attempt === MAIN_FLOW_MAX_ATTEMPTS) {
        throw error;
      }
      console.error(
        `Main flow failed (attempt ${attempt}/${MAIN_FLOW_MAX_ATTEMPTS}): ${error?.message || error}`,
      );
      console.error(`Retrying full flow in ${Math.ceil(MAIN_FLOW_RETRY_WAIT_MS / 1000)} seconds...`);
      await new Promise((resolve) => setTimeout(resolve, MAIN_FLOW_RETRY_WAIT_MS));
    }
  }
}

runMainWithRetries().catch((error) => {
  console.error(error.message);
  process.exit(1);
});

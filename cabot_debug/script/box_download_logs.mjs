#!/usr/bin/env node

import fs from "fs";
import os from "os";
import path from "path";
import readline from "readline";
import { spawn } from "child_process";
import { fileURLToPath } from "url";
import { chromium } from "playwright";

const SCRIPT_DIR = path.dirname(fileURLToPath(import.meta.url));
const REPO_ROOT = path.resolve(SCRIPT_DIR, "..", "..");
const PROFILE_DIR = path.join(REPO_ROOT, "docker", "home", ".box-profile");
const DOWNLOAD_DIR = path.join(os.homedir(), "Downloads");
const GITHUB_LOGIN_MARKER = path.join(PROFILE_DIR, ".github-login-complete");
const BOX_LOGIN_MARKER = path.join(PROFILE_DIR, ".box-login-complete");
const DOWNLOAD_HELPER = path.join(SCRIPT_DIR, "download-helper.sh");
const BOX_HOST_PATTERN = /box\.com/i;
const CHROMIUM_PROFILE_LOCK_FILES = ["SingletonLock", "SingletonCookie", "SingletonSocket"];

function ensureDir(dirPath) {
  fs.mkdirSync(dirPath, { recursive: true });
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

function ask(question) {
  const rl = readline.createInterface({ input: process.stdin, output: process.stdout });
  return new Promise((resolve) => {
    rl.question(question, () => {
      rl.close();
      resolve();
    });
  });
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

async function extractFirstCommentBoxLinks(page, issueUrl) {
  await page.goto(issueUrl, { waitUntil: "domcontentloaded" });
  await waitForStableDom(page);

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

  throw new Error("No Box links found in the first issue comment/body.");
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
  if (/box\.com\/login/i.test(page.url())) {
    return true;
  }

  const loginSelectors = [
    "input[type='email']",
    "input[name='login']",
    "input[type='password']",
    "a:has-text('Log in')",
    "a:has-text('Sign in')",
    "button:has-text('Log In')",
    "button:has-text('Sign In')",
  ];

  for (const selector of loginSelectors) {
    if (await page.locator(selector).first().isVisible({ timeout: 1000 }).catch(() => false)) {
      return true;
    }
  }

  return false;
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
    console.log(`Using existing ${serviceName} login session from persistent profile.`);
    await ask(`Log in to ${serviceName} in the opened browser if prompted, then press Enter to continue...`);
    return;
  }

  console.log(`First ${serviceName} login detected.`);
  console.log(`Please log in to ${serviceName} manually in the opened browser.`);
  if (promptUrl) {
    console.log(`Target page: ${promptUrl}`);
  }
  await ask("Press Enter to continue...");
  fs.writeFileSync(markerPath, `${new Date().toISOString()}\n`);
  await page.waitForTimeout(1000);
}

async function downloadSharedBoxFile(page, item) {
  await page.goto(item.href, { waitUntil: "domcontentloaded" });
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
  const destination = path.join(DOWNLOAD_DIR, filename);
  await download.saveAs(destination);
  console.log(`Downloaded: ${filename}`);
}

async function downloadSharedBoxFiles(context, items, concurrency = 10) {
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

async function resolveFileLinksFromFolder(page, folderUrl, targetNames) {
  await page.goto(folderUrl, { waitUntil: "domcontentloaded" });
  await waitForStableDom(page);

  const resolved = [];
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
      throw new Error(`Could not find ${targetName} in Box folder view.`);
    }

    resolved.push({ name: targetName, href });
  }

  return resolved;
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

  return new Promise((resolve, reject) => {
    child.on("error", reject);
    child.on("exit", (code) => {
      if (code === 0) {
        resolve();
        return;
      }
      reject(new Error(`download-helper exited with code ${code}`));
    });
  });
}

async function main() {
  const issueUrl = process.argv[2];
  if (!issueUrl) {
    console.error("Usage: node script/box_download_logs.mjs <github-issue-url>");
    process.exit(1);
  }

  ensureDir(PROFILE_DIR);
  ensureDir(DOWNLOAD_DIR);

  const issueInfo = getIssueInfo(issueUrl);
  const context = await launchPersistentContextWithRetry();

  try {
    const page = context.pages()[0] || await context.newPage();
    await page.goto(issueUrl, { waitUntil: "domcontentloaded" });
    await maybeCompleteLogin(page, GITHUB_LOGIN_MARKER, "GitHub", issueUrl, pageNeedsGitHubLogin);

    const boxLinks = await extractFirstCommentBoxLinks(page, issueUrl);
    const filesToDownload = boxLinks.filter((item) => /\/s\/|\/file\/|\/folder\//i.test(item.href));
    const { prefix, partCount } = getPrefixAndPartCount(filesToDownload);
    const { folderLink, targetNames } = getFolderAndTargetNames(filesToDownload);

    console.log(`Issue: ${issueInfo.owner}/${issueInfo.repo}#${issueInfo.issueNumber}`);
    console.log(`Prefix: ${prefix}`);
    console.log(`Parts: ${partCount}`);

    cleanupStaleDownloads(prefix);
    const helperPromise = startDownloadHelper(prefix, partCount, issueInfo.issueTag);

    if (!folderLink) {
      throw new Error("No Box folder link found in the first comment.");
    }
    if (targetNames.length === 0) {
      throw new Error("No target filenames found in the first comment.");
    }

    await page.goto(folderLink.href, { waitUntil: "domcontentloaded" });
    await maybeCompleteLogin(page, BOX_LOGIN_MARKER, "Box", folderLink.href, pageNeedsBoxLogin);

    const fileLinks = await resolveFileLinksFromFolder(page, folderLink.href, targetNames);
    await downloadSharedBoxFiles(context, fileLinks);

    await helperPromise;
    console.log(`Extracted under docker/home/sandbox/${issueInfo.issueTag}`);
  } finally {
    await context.close();
  }
}

main().catch((error) => {
  console.error(error.message);
  process.exit(1);
});

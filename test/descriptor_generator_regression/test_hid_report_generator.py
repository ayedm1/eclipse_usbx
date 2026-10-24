"""Browser regression tests for usbx_hid_report_descriptor_generator.html.

Every template/usb_hid_report_descriptor_*.json file is loaded into the page and
the generated outputs are checked for consistency.
Set BROWSER_CHANNEL=msedge to use an installed Edge instead of bundled Chromium.
"""
import json
import os
import re
from pathlib import Path

import pytest
from playwright.sync_api import sync_playwright

GENERATOR_DIR = Path(__file__).resolve().parents[2] / "device_descriptor_generator"
PAGE = GENERATOR_DIR / "usbx_hid_report_descriptor_generator.html"
TEMPLATES = sorted((GENERATOR_DIR / "template").glob("usb_hid_report_descriptor_*.json"))
TEMPLATE_IDS = [t.stem.replace("usb_hid_report_descriptor_", "") for t in TEMPLATES]


@pytest.fixture(scope="module")
def browser():
    with sync_playwright() as p:
        channel = os.environ.get("BROWSER_CHANNEL") or None
        b = p.chromium.launch(channel=channel)
        yield b
        b.close()


@pytest.fixture()
def page(browser):
    pg = browser.new_page()
    errors = []
    pg.on("pageerror", lambda e: errors.append(str(e)))
    pg.on("dialog", lambda d: d.accept())
    pg.goto(PAGE.as_uri())
    pg.wait_for_load_state("load")
    pg.errors = errors
    yield pg
    pg.close()


def load_descriptor(page, descriptor):
    page.evaluate(
        """(d) => { currentDescriptor = d; renderReportItems(); generateOutput(); }""",
        descriptor,
    )


def test_templates_found():
    assert len(TEMPLATES) >= 7


def test_page_loads_without_errors(page):
    assert page.title()
    assert page.errors == []


@pytest.mark.parametrize("template", TEMPLATES, ids=TEMPLATE_IDS)
def test_template_json_is_valid(template):
    data = json.loads(template.read_text(encoding="utf-8"))
    assert isinstance(data.get("descriptor"), list)
    assert data.get("deviceType")


@pytest.mark.parametrize("template", TEMPLATES, ids=TEMPLATE_IDS)
def test_template_generates_consistent_outputs(page, template):
    data = json.loads(template.read_text(encoding="utf-8"))
    if not data["descriptor"]:
        pytest.skip("empty (custom) template")

    load_descriptor(page, data["descriptor"])
    c_text = page.inner_text("#outputC")
    hex_text = page.inner_text("#outputHex")
    readable = page.inner_text("#outputReadable")

    length = int(re.search(r"UX_HID_REPORT_DESCRIPTOR_LENGTH (\d+)", c_text).group(1))
    c_bytes = [
        b
        for line in c_text.split("{", 1)[1].splitlines()
        for b in re.findall(r"0x[0-9A-F]{2}", line.split("//")[0])
    ]
    hex_bytes = [
        b
        for line in hex_text.splitlines()
        for b in line.split(":", 1)[1].split()
    ]

    assert len(c_bytes) == length
    assert [b[2:] for b in c_bytes] == hex_bytes
    assert page.inner_text("#itemCount") == str(len(data["descriptor"]))
    assert page.inner_text("#totalSize") == f"{length} bytes"
    assert len(readable.strip().splitlines()) == len(data["descriptor"])

    # Balanced collections and the top-level application collection.
    tags = [item["tag"] for item in data["descriptor"]]
    assert tags.count("COLLECTION") == tags.count("END_COLLECTION")
    assert page.errors == []


@pytest.mark.parametrize("template", TEMPLATES, ids=TEMPLATE_IDS)
def test_template_matches_builtin_device_type(page, template):
    data = json.loads(template.read_text(encoding="utf-8"))
    device_type = data["deviceType"]
    if not data["descriptor"]:
        pytest.skip("empty (custom) template")

    builtin = page.evaluate(
        """async (t) => {
            document.getElementById('deviceType').value = t;
            await loadDeviceTemplate();
            return currentDescriptor;
        }""",
        device_type,
    )
    assert builtin, f"no descriptor for device type {device_type}"

    load_descriptor(page, data["descriptor"])
    from_json = page.inner_text("#outputC")
    load_descriptor(page, builtin)
    assert page.inner_text("#outputC") == from_json


def test_device_type_select_loads_descriptor(page):
    page.evaluate(
        """async () => {
            document.getElementById('deviceType').value = 'keyboard';
            await loadDeviceTemplate();
            generateOutput();
        }"""
    )
    assert "UX_HID_REPORT_DESCRIPTOR_LENGTH" in page.inner_text("#outputC")
    assert int(page.inner_text("#itemCount")) > 0


def test_new_descriptor_clears_items(page):
    page.evaluate("newDescriptor()")
    assert page.evaluate("currentDescriptor.length") == 0
    assert page.input_value("#deviceType") == "custom"


def test_add_and_delete_report_item(page):
    before = page.evaluate("currentDescriptor.length")
    page.evaluate("addReportItem()")
    assert page.evaluate("currentDescriptor.length") == before + 1
    page.evaluate("deleteReportItem(currentDescriptor.length - 1)")
    assert page.evaluate("currentDescriptor.length") == before
    assert page.errors == []

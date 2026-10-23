"""Browser regression tests for usbx_device_descriptor_generator.html.

Run:  pip install pytest playwright && playwright install chromium
      pytest test/descriptor_generator_regression -v
Set BROWSER_CHANNEL=msedge to use an installed Edge instead of bundled Chromium.
"""
import json
import os
from pathlib import Path

import pytest
from playwright.sync_api import sync_playwright

GENERATOR_DIR = Path(__file__).resolve().parents[2] / "device_descriptor_generator"
PAGE = GENERATOR_DIR / "usbx_device_descriptor_generator.html"
TEMPLATES = sorted((GENERATOR_DIR / "template").glob("usb_device_descriptor_*.json"))


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
    pg.goto(PAGE.as_uri())
    pg.wait_for_load_state("load")
    pg.errors = errors
    yield pg
    pg.close()


def enable(page, cls, count=1):
    page.evaluate(
        """([cls, count]) => {
            const cb = document.getElementById('class-' + cls);
            cb.checked = true;
            toggleUSBClass(cls);
            const n = document.getElementById('class-num-' + cls);
            n.value = count;
            n.dispatchEvent(new Event('change', { bubbles: true }));
        }""",
        [cls, count],
    )


def test_page_loads_without_errors(page):
    assert page.title()
    assert page.errors == []


def test_class_rows_visibility(page):
    visible = page.evaluate(
        "['cdc','dfu','hid'].map(c => !document.getElementById('class-' + c).closest('tr').hidden)"
    )
    assert visible == [True, True, True]
    assert page.evaluate("document.getElementById('class-printer').closest('tr').hidden")


def test_reorder_note_present(page):
    assert "To reorder classes instances numbers click" in page.content()


def test_dfu_defaults_to_first_position(page):
    enable(page, "hid", 2)
    enable(page, "dfu")
    order = page.evaluate("getInstanceOrder().map(x => x.className + x.instance)")
    assert order == ["dfu1", "hid1", "hid2"]


def test_move_instance_changes_order(page):
    enable(page, "hid", 2)
    enable(page, "dfu")
    first = page.evaluate("getInstanceOrder()[0]")
    page.evaluate("(() => { const o = getInstanceOrder()[1]; moveClassInstance(o.className, o.instance, -1); })()")
    assert page.evaluate("getInstanceOrder()[0]") != first


def test_dfu_interface_string_defaults(page):
    enable(page, "dfu")
    assert page.input_value("#dfu-config-1-dfuIInterface") == "0"
    assert page.input_value("#dfu-config-1-dfuInterfaceString") == "DFU interface"
    assert page.is_disabled("#dfu-config-1-dfuInterfaceString")
    assert page.locator("#dfu-config-1-languageIdDisplay").count() == 1


def test_hid_descriptor_card_title(page):
    enable(page, "hid")
    assert "HID Descriptor" in page.content()


def test_cdc_interface_descriptor_card_and_string(page):
    enable(page, "cdc")
    assert page.locator("#cdc-config-fields-1 #cdc-config-1-cdcIInterface").count() == 1
    assert page.locator("#cdc-func-fields-1 #cdc-config-1-cdcBcdCDC").count() == 1
    assert page.locator("#cdc-config-1-languageIdDisplay").count() == 1
    assert page.input_value("#cdc-config-1-cdcInterfaceString") == "CDC ACM interface"
    assert page.is_disabled("#cdc-config-1-cdcInterfaceString")

    page.fill("#cdc-config-1-cdcIInterface", "5")
    page.dispatch_event("#cdc-config-1-cdcIInterface", "change")
    assert page.is_enabled("#cdc-config-1-cdcInterfaceString")
    assert page.evaluate("readCDCConfigValues(1).cdcIInterface") == "5"


@pytest.mark.parametrize("template", TEMPLATES, ids=lambda p: p.name)
def test_template_json_round_trip(page, template):
    """Load a template through the page's Load JSON flow and save it back unchanged."""
    assert TEMPLATES, "no template JSON files found"

    with page.expect_file_chooser() as chooser:
        page.evaluate("loadJSON()")
    chooser.value.set_files(str(template))
    page.wait_for_function(
        "() => document.getElementById('class-hid').checked || document.querySelector('[id^=class-]:checked')"
    )
    page.wait_for_timeout(500)

    with page.expect_download() as download:
        page.evaluate("saveJSON()")
    saved = json.loads(Path(download.value.path()).read_text(encoding="utf-8"))
    expected = json.loads(template.read_text(encoding="utf-8"))

    assert page.errors == []
    assert saved == expected

def test_json_code_tab_shows_json(page):
    enable(page, "dfu")
    page.click("text=JSON Code")
    assert page.is_visible("#json-array-output")
    shown = json.loads(page.inner_text("#json-array-output"))
    assert "dfu" in shown["classes"]


@pytest.mark.parametrize(
    "tab,output_id,filename",
    [
        ("c-array", "c-array-output", "ux_device_descriptor.c"),
        ("json-array", "json-array-output", "ux_device_descriptor.json"),
        ("hex", "hex-output", "ux_device_descriptor_hex.txt"),
        ("readable", "readable-output", "ux_device_descriptor_readable.txt"),
    ],
)
def test_save_buttons_use_expected_filenames(page, tab, output_id, filename):
    enable(page, "dfu")
    page.evaluate("generateDescriptor()")
    page.evaluate(f"switchTab('{tab}', document.querySelector('.tab-button'))")
    with page.expect_download() as download:
        page.click(f"#{tab} .output-actions button:has-text('Save')")
    assert download.value.suggested_filename == filename
    saved = Path(download.value.path()).read_text(encoding="utf-8")
    assert saved == page.inner_text(f"#{output_id}")

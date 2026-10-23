// HID Report Descriptor Generator Script

// HID Item Type Constants
const HID_ITEM_TYPE = {
    MAIN: 0,
    GLOBAL: 1,
    LOCAL: 2
};

// HID Main Item Tags
const HID_MAIN_ITEM = {
    INPUT: 0x80,
    OUTPUT: 0x90,
    FEATURE: 0xB0,
    COLLECTION: 0xA0,
    END_COLLECTION: 0xC0
};

// HID Global Item Tags
const HID_GLOBAL_ITEM = {
    USAGE_PAGE: 0x04,
    LOGICAL_MINIMUM: 0x14,
    LOGICAL_MAXIMUM: 0x24,
    PHYSICAL_MINIMUM: 0x34,
    PHYSICAL_MAXIMUM: 0x44,
    UNIT_EXPONENT: 0x54,
    UNIT: 0x64,
    REPORT_SIZE: 0x74,
    REPORT_ID: 0x84,
    REPORT_COUNT: 0x94,
    PUSH: 0xA4,
    POP: 0xB4
};

// HID Local Item Tags
const HID_LOCAL_ITEM = {
    USAGE: 0x08,
    USAGE_MINIMUM: 0x18,
    USAGE_MAXIMUM: 0x28,
    DESIGNATOR_INDEX: 0x38,
    DESIGNATOR_MINIMUM: 0x48,
    DESIGNATOR_MAXIMUM: 0x58,
    STRING_INDEX: 0x78,
    STRING_MINIMUM: 0x88,
    STRING_MAXIMUM: 0x98,
    DELIMITER: 0xA8
};

// HID Usage Pages
const HID_USAGE_PAGE = {
    GENERIC_DESKTOP: 0x01,
    SIMULATION: 0x02,
    VR: 0x03,
    SPORT: 0x04,
    GAME: 0x05,
    GENERIC_DEVICE: 0x06,
    KEYBOARD: 0x07,
    LED: 0x08,
    BUTTON: 0x09,
    ORDINAL: 0x0A,
    TELEPHONY: 0x0B,
    CONSUMER: 0x0C,
    DIGITIZER: 0x0D,
    PID: 0x0F,
    UNICODE: 0x10,
    VENDOR_DEFINED: 0xFF00
};

// Report Descriptor Templates
const TEMPLATES = {
    'mouse': {
        name: 'Mouse (Boot Protocol)',
        descriptor: [
            { tag: 'USAGE_PAGE', value: 0x01, comment: 'Generic Desktop' },
            { tag: 'USAGE', value: 0x02, comment: 'Mouse' },
            { tag: 'COLLECTION', value: 0x01, comment: 'Application' },
            { tag: 'USAGE', value: 0x01, comment: 'Pointer' },
            { tag: 'COLLECTION', value: 0x00, comment: 'Physical' },
            // Buttons
            { tag: 'USAGE_PAGE', value: 0x09, comment: 'Button' },
            { tag: 'USAGE_MINIMUM', value: 0x01, comment: 'Button 1' },
            { tag: 'USAGE_MAXIMUM', value: 0x03, comment: 'Button 3' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x01, comment: 'Logical Max (1)' },
            { tag: 'REPORT_SIZE', value: 0x01, comment: '1 bit' },
            { tag: 'REPORT_COUNT', value: 0x03, comment: '3 buttons' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            // Padding
            { tag: 'REPORT_SIZE', value: 0x05, comment: '5 bits' },
            { tag: 'REPORT_COUNT', value: 0x01, comment: '1 report' },
            { tag: 'INPUT', value: 0x03, comment: 'Constant, Variable, Absolute (Padding)' },
            // X, Y
            { tag: 'USAGE_PAGE', value: 0x01, comment: 'Generic Desktop' },
            { tag: 'USAGE', value: 0x30, comment: 'X' },
            { tag: 'USAGE', value: 0x31, comment: 'Y' },
            { tag: 'LOGICAL_MINIMUM', value: 0x81, comment: 'Logical Min (-127)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x7F, comment: 'Logical Max (127)' },
            { tag: 'REPORT_SIZE', value: 0x08, comment: '8 bits' },
            { tag: 'REPORT_COUNT', value: 0x02, comment: '2 axes (X, Y)' },
            { tag: 'INPUT', value: 0x06, comment: 'Data, Variable, Relative' },
            // Wheel
            { tag: 'USAGE', value: 0x38, comment: 'Wheel' },
            { tag: 'LOGICAL_MINIMUM', value: 0x81, comment: 'Logical Min (-127)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x7F, comment: 'Logical Max (127)' },
            { tag: 'REPORT_SIZE', value: 0x08, comment: '8 bits' },
            { tag: 'REPORT_COUNT', value: 0x01, comment: '1 wheel' },
            { tag: 'INPUT', value: 0x06, comment: 'Data, Variable, Relative' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Physical' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Application' }
        ]
    },
    'mouse-absolute': {
        name: 'Mouse (Absolute Positioning)',
        descriptor: [
            { tag: 'USAGE_PAGE', value: 0x01, comment: 'Generic Desktop' },
            { tag: 'USAGE', value: 0x02, comment: 'Mouse' },
            { tag: 'COLLECTION', value: 0x01, comment: 'Application' },
            { tag: 'USAGE', value: 0x01, comment: 'Pointer' },
            { tag: 'COLLECTION', value: 0x00, comment: 'Physical' },
            // Buttons
            { tag: 'USAGE_PAGE', value: 0x09, comment: 'Button' },
            { tag: 'USAGE_MINIMUM', value: 0x01, comment: 'Button 1' },
            { tag: 'USAGE_MAXIMUM', value: 0x03, comment: 'Button 3' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x01, comment: 'Logical Max (1)' },
            { tag: 'REPORT_SIZE', value: 0x01, comment: '1 bit' },
            { tag: 'REPORT_COUNT', value: 0x03, comment: '3 buttons' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            // Padding
            { tag: 'REPORT_SIZE', value: 0x05, comment: '5 bits' },
            { tag: 'REPORT_COUNT', value: 0x01, comment: '1 report' },
            { tag: 'INPUT', value: 0x03, comment: 'Constant, Variable, Absolute (Padding)' },
            // X, Y (Absolute)
            { tag: 'USAGE_PAGE', value: 0x01, comment: 'Generic Desktop' },
            { tag: 'USAGE', value: 0x30, comment: 'X' },
            { tag: 'USAGE', value: 0x31, comment: 'Y' },
            { tag: 'LOGICAL_MINIMUM', value: 0x0000, comment: 'Logical Min (0)', size: 2 },
            { tag: 'LOGICAL_MAXIMUM', value: 0x7FFF, comment: 'Logical Max (32767)', size: 2 },
            { tag: 'REPORT_SIZE', value: 0x10, comment: '16 bits' },
            { tag: 'REPORT_COUNT', value: 0x02, comment: '2 axes (X, Y)' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            // Wheel
            { tag: 'USAGE', value: 0x38, comment: 'Wheel' },
            { tag: 'LOGICAL_MINIMUM', value: 0x81, comment: 'Logical Min (-127)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x7F, comment: 'Logical Max (127)' },
            { tag: 'REPORT_SIZE', value: 0x08, comment: '8 bits' },
            { tag: 'REPORT_COUNT', value: 0x01, comment: '1 wheel' },
            { tag: 'INPUT', value: 0x06, comment: 'Data, Variable, Relative' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Physical' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Application' }
        ]
    },
    'keyboard': {
        name: 'Keyboard (Boot Protocol)',
        descriptor: [
            { tag: 'USAGE_PAGE', value: 0x01, comment: 'Generic Desktop' },
            { tag: 'USAGE', value: 0x06, comment: 'Keyboard' },
            { tag: 'COLLECTION', value: 0x01, comment: 'Application' },
            // Modifier Keys
            { tag: 'USAGE_PAGE', value: 0x07, comment: 'Keyboard/Keypad' },
            { tag: 'USAGE_MINIMUM', value: 0xE0, comment: 'Left Control' },
            { tag: 'USAGE_MAXIMUM', value: 0xE7, comment: 'Right GUI' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x01, comment: 'Logical Max (1)' },
            { tag: 'REPORT_SIZE', value: 0x01, comment: '1 bit' },
            { tag: 'REPORT_COUNT', value: 0x08, comment: '8 modifiers' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            // Reserved Byte
            { tag: 'REPORT_SIZE', value: 0x08, comment: '8 bits' },
            { tag: 'REPORT_COUNT', value: 0x01, comment: '1 byte' },
            { tag: 'INPUT', value: 0x01, comment: 'Constant (Reserved)' },
            // LED Output Report
            { tag: 'USAGE_PAGE', value: 0x08, comment: 'LED' },
            { tag: 'USAGE_MINIMUM', value: 0x01, comment: 'Num Lock' },
            { tag: 'USAGE_MAXIMUM', value: 0x05, comment: 'Kana' },
            { tag: 'REPORT_SIZE', value: 0x01, comment: '1 bit' },
            { tag: 'REPORT_COUNT', value: 0x05, comment: '5 LEDs' },
            { tag: 'OUTPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            // LED Padding
            { tag: 'REPORT_SIZE', value: 0x03, comment: '3 bits' },
            { tag: 'REPORT_COUNT', value: 0x01, comment: '1 report' },
            { tag: 'OUTPUT', value: 0x01, comment: 'Constant (Padding)' },
            // Keycodes
            { tag: 'USAGE_PAGE', value: 0x07, comment: 'Keyboard/Keypad' },
            { tag: 'USAGE_MINIMUM', value: 0x00, comment: 'No Event' },
            { tag: 'USAGE_MAXIMUM', value: 0x65, comment: 'Keyboard Application' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x65, comment: 'Logical Max (101)' },
            { tag: 'REPORT_SIZE', value: 0x08, comment: '8 bits' },
            { tag: 'REPORT_COUNT', value: 0x06, comment: '6 keycodes' },
            { tag: 'INPUT', value: 0x00, comment: 'Data, Array' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Application' }
        ]
    },
    'gamepad': {
        name: 'Game Controller / Gamepad',
        descriptor: [
            { tag: 'USAGE_PAGE', value: 0x01, comment: 'Generic Desktop' },
            { tag: 'USAGE', value: 0x05, comment: 'Game Pad' },
            { tag: 'COLLECTION', value: 0x01, comment: 'Application' },
            // Buttons
            { tag: 'USAGE_PAGE', value: 0x09, comment: 'Button' },
            { tag: 'USAGE_MINIMUM', value: 0x01, comment: 'Button 1' },
            { tag: 'USAGE_MAXIMUM', value: 0x10, comment: 'Button 16' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x01, comment: 'Logical Max (1)' },
            { tag: 'REPORT_SIZE', value: 0x01, comment: '1 bit' },
            { tag: 'REPORT_COUNT', value: 0x10, comment: '16 buttons' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            // X, Y, Z, Rz (Analog sticks)
            { tag: 'USAGE_PAGE', value: 0x01, comment: 'Generic Desktop' },
            { tag: 'USAGE', value: 0x30, comment: 'X' },
            { tag: 'USAGE', value: 0x31, comment: 'Y' },
            { tag: 'USAGE', value: 0x32, comment: 'Z' },
            { tag: 'USAGE', value: 0x35, comment: 'Rz' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0xFF, comment: 'Logical Max (255)' },
            { tag: 'REPORT_SIZE', value: 0x08, comment: '8 bits' },
            { tag: 'REPORT_COUNT', value: 0x04, comment: '4 axes' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            // Hat Switch
            { tag: 'USAGE', value: 0x39, comment: 'Hat Switch' },
            { tag: 'LOGICAL_MINIMUM', value: 0x01, comment: 'Logical Min (1)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x08, comment: 'Logical Max (8)' },
            { tag: 'PHYSICAL_MINIMUM', value: 0x00, comment: 'Physical Min (0)' },
            { tag: 'PHYSICAL_MAXIMUM', value: 0x13B, comment: 'Physical Max (315)', size: 2 },
            { tag: 'UNIT', value: 0x14, comment: 'Unit (Degrees)' },
            { tag: 'REPORT_SIZE', value: 0x04, comment: '4 bits' },
            { tag: 'REPORT_COUNT', value: 0x01, comment: '1 hat' },
            { tag: 'INPUT', value: 0x42, comment: 'Data, Variable, Absolute, Null State' },
            // Padding
            { tag: 'UNIT', value: 0x00, comment: 'Unit (None)' },
            { tag: 'REPORT_SIZE', value: 0x04, comment: '4 bits' },
            { tag: 'REPORT_COUNT', value: 0x01, comment: '1 report' },
            { tag: 'INPUT', value: 0x01, comment: 'Constant (Padding)' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Application' }
        ]
    },
    'joystick': {
        name: 'Joystick',
        descriptor: [
            { tag: 'USAGE_PAGE', value: 0x01, comment: 'Generic Desktop' },
            { tag: 'USAGE', value: 0x04, comment: 'Joystick' },
            { tag: 'COLLECTION', value: 0x01, comment: 'Application' },
            { tag: 'USAGE', value: 0x01, comment: 'Pointer' },
            { tag: 'COLLECTION', value: 0x00, comment: 'Physical' },
            // X, Y axes
            { tag: 'USAGE', value: 0x30, comment: 'X' },
            { tag: 'USAGE', value: 0x31, comment: 'Y' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0xFF, comment: 'Logical Max (255)' },
            { tag: 'REPORT_SIZE', value: 0x08, comment: '8 bits' },
            { tag: 'REPORT_COUNT', value: 0x02, comment: '2 axes' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Physical' },
            // Buttons
            { tag: 'USAGE_PAGE', value: 0x09, comment: 'Button' },
            { tag: 'USAGE_MINIMUM', value: 0x01, comment: 'Button 1' },
            { tag: 'USAGE_MAXIMUM', value: 0x08, comment: 'Button 8' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x01, comment: 'Logical Max (1)' },
            { tag: 'REPORT_SIZE', value: 0x01, comment: '1 bit' },
            { tag: 'REPORT_COUNT', value: 0x08, comment: '8 buttons' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Application' }
        ]
    },
    'consumer': {
        name: 'Consumer Control',
        descriptor: [
            { tag: 'USAGE_PAGE', value: 0x0C, comment: 'Consumer' },
            { tag: 'USAGE', value: 0x01, comment: 'Consumer Control' },
            { tag: 'COLLECTION', value: 0x01, comment: 'Application' },
            // Media keys
            { tag: 'USAGE_PAGE', value: 0x0C, comment: 'Consumer' },
            { tag: 'USAGE', value: 0xE2, comment: 'Mute' },
            { tag: 'USAGE', value: 0xE9, comment: 'Volume Increment' },
            { tag: 'USAGE', value: 0xEA, comment: 'Volume Decrement' },
            { tag: 'USAGE', value: 0xCD, comment: 'Play/Pause' },
            { tag: 'USAGE', value: 0xB5, comment: 'Scan Next Track' },
            { tag: 'USAGE', value: 0xB6, comment: 'Scan Previous Track' },
            { tag: 'USAGE', value: 0xB7, comment: 'Stop' },
            { tag: 'USAGE', value: 0xB8, comment: 'Eject' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0x01, comment: 'Logical Max (1)' },
            { tag: 'REPORT_SIZE', value: 0x01, comment: '1 bit' },
            { tag: 'REPORT_COUNT', value: 0x08, comment: '8 controls' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Application' }
        ]
    },
    'vendor': {
        name: 'Vendor-Specific',
        descriptor: [
            { tag: 'USAGE_PAGE', value: 0xFF00, comment: 'Vendor-Defined', size: 2 },
            { tag: 'USAGE', value: 0x01, comment: 'Vendor Usage 1' },
            { tag: 'COLLECTION', value: 0x01, comment: 'Application' },
            // Input Report
            { tag: 'USAGE', value: 0x02, comment: 'Vendor Usage 2' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0xFF, comment: 'Logical Max (255)' },
            { tag: 'REPORT_SIZE', value: 0x08, comment: '8 bits' },
            { tag: 'REPORT_COUNT', value: 0x40, comment: '64 bytes' },
            { tag: 'INPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            // Output Report
            { tag: 'USAGE', value: 0x03, comment: 'Vendor Usage 3' },
            { tag: 'LOGICAL_MINIMUM', value: 0x00, comment: 'Logical Min (0)' },
            { tag: 'LOGICAL_MAXIMUM', value: 0xFF, comment: 'Logical Max (255)' },
            { tag: 'REPORT_SIZE', value: 0x08, comment: '8 bits' },
            { tag: 'REPORT_COUNT', value: 0x40, comment: '64 bytes' },
            { tag: 'OUTPUT', value: 0x02, comment: 'Data, Variable, Absolute' },
            { tag: 'END_COLLECTION', value: 0x00, comment: 'End Application' }
        ]
    },
    'custom': {
        name: 'Custom',
        descriptor: []
    }
};

const TEMPLATE_FILE_MAP = {
    'custom': 'Custom.json',
    'mouse': 'mouse_boot.json',
    'mouse-absolute': 'mouse_absolute.json',
    'keyboard': 'keyboard.json',
    'gamepad': 'game_pad.json',
    'joystick': 'Joystick.json',
    'consumer': 'Consumer.json',
    'vendor': 'Vendor_specific.json'
};

// Tag name to tag code mapping
const TAG_MAP = {
    // Main Items
    'INPUT': 0x80,
    'OUTPUT': 0x90,
    'FEATURE': 0xB0,
    'COLLECTION': 0xA0,
    'END_COLLECTION': 0xC0,
    // Global Items
    'USAGE_PAGE': 0x04,
    'LOGICAL_MINIMUM': 0x14,
    'LOGICAL_MAXIMUM': 0x24,
    'PHYSICAL_MINIMUM': 0x34,
    'PHYSICAL_MAXIMUM': 0x44,
    'UNIT_EXPONENT': 0x54,
    'UNIT': 0x64,
    'REPORT_SIZE': 0x74,
    'REPORT_ID': 0x84,
    'REPORT_COUNT': 0x94,
    'PUSH': 0xA4,
    'POP': 0xB4,
    // Local Items
    'USAGE': 0x08,
    'USAGE_MINIMUM': 0x18,
    'USAGE_MAXIMUM': 0x28,
    'DESIGNATOR_INDEX': 0x38,
    'DESIGNATOR_MINIMUM': 0x48,
    'DESIGNATOR_MAXIMUM': 0x58,
    'STRING_INDEX': 0x78,
    'STRING_MINIMUM': 0x88,
    'STRING_MAXIMUM': 0x98,
    'DELIMITER': 0xA8
};

// Reverse mapping for tag code to name
const TAG_NAME_MAP = Object.fromEntries(
    Object.entries(TAG_MAP).map(([key, value]) => [value, key])
);

// Current report descriptor
let currentDescriptor = [];
let reportItemIdCounter = 0;
const templateCache = {};

async function loadTemplateFromJson(deviceType) {
    const fileName = TEMPLATE_FILE_MAP[deviceType];
    if (!fileName) {
        return null;
    }

    if (templateCache[deviceType]) {
        return templateCache[deviceType];
    }

    const response = await fetch(`hid_report_descriptor_template/${fileName}`, {
        cache: 'no-cache'
    });

    if (!response.ok) {
        throw new Error(`Failed to load template file: ${fileName}`);
    }

    const data = await response.json();
    if (!data || !Array.isArray(data.descriptor)) {
        throw new Error(`Invalid template JSON format: ${fileName}`);
    }

    templateCache[deviceType] = data;
    return data;
}

function getItemDataSize(item) {
    if (typeof item.size === 'number') {
        return item.size;
    }

    if (item.tag === 'END_COLLECTION' || item.tag === 'PUSH' || item.tag === 'POP') {
        return 0;
    }

    const value = item.value || 0;
    if (value > 0xFFFF) {
        return 4;
    }
    if (value > 0xFF) {
        return 2;
    }

    return 1;
}

// Initialize on page load
document.addEventListener('DOMContentLoaded', function() {
    loadDeviceTemplate();
});

// Load device template based on selection
async function loadDeviceTemplate() {
    const deviceType = document.getElementById('deviceType').value;

    try {
        const jsonTemplate = await loadTemplateFromJson(deviceType);
        if (jsonTemplate && Array.isArray(jsonTemplate.descriptor)) {
            currentDescriptor = JSON.parse(JSON.stringify(jsonTemplate.descriptor));
            renderReportItems();
            return true;
        }
    } catch (error) {
        console.warn('JSON template load failed, using fallback template:', error);
    }

    const template = TEMPLATES[deviceType];

    if (template) {
        currentDescriptor = JSON.parse(JSON.stringify(template.descriptor));
        renderReportItems();
        return true;
    }

    return false;
}

// Render report items in the form
function renderReportItems() {
    const container = document.getElementById('reportItemsContainer');
    container.innerHTML = '';

    if (currentDescriptor.length === 0) {
        container.innerHTML = '<p style="color: #666; font-style: italic;">No items defined. Click "Add Item" to start building your descriptor.</p>';
        return;
    }

    currentDescriptor.forEach((item, index) => {
        const itemDiv = document.createElement('div');
        itemDiv.className = 'report-item';
        itemDiv.innerHTML = `
            <div class="report-item-header">
                <span class="report-item-index">#${index + 1}</span>
                <select class="report-item-tag" data-index="${index}" onchange="updateItemTag(${index})">
                    ${Object.keys(TAG_MAP).map(tag =>
                        `<option value="${tag}" ${item.tag === tag ? 'selected' : ''}>${tag}</option>`
                    ).join('')}
                </select>
                <input type="text" class="report-item-value" data-index="${index}"
                       value="0x${item.value.toString(16).toUpperCase().padStart(item.size === 2 ? 4 : 2, '0')}"
                       onchange="updateItemValue(${index})" placeholder="0x00">
                <input type="text" class="report-item-comment" data-index="${index}"
                       value="${item.comment || ''}" onchange="updateItemComment(${index})" placeholder="Comment">
                <button type="button" class="btn-delete" onclick="deleteReportItem(${index})" title="Delete Item">×</button>
            </div>
        `;
        container.appendChild(itemDiv);
    });
}

// Add new report item
function addReportItem() {
    currentDescriptor.push({
        tag: 'USAGE_PAGE',
        value: 0x01,
        comment: 'New Item',
        size: 1
    });
    renderReportItems();
}

// Update item tag
function updateItemTag(index) {
    const select = document.querySelector(`.report-item-tag[data-index="${index}"]`);
    currentDescriptor[index].tag = select.value;
}

// Update item value
function updateItemValue(index) {
    const input = document.querySelector(`.report-item-value[data-index="${index}"]`);
    let value = input.value.trim();

    // Parse hex or decimal
    if (value.startsWith('0x') || value.startsWith('0X')) {
        currentDescriptor[index].value = parseInt(value.substring(2), 16);
    } else {
        currentDescriptor[index].value = parseInt(value, 10);
    }

    // Determine size based on value
    if (currentDescriptor[index].value > 0xFF) {
        currentDescriptor[index].size = 2;
    } else {
        currentDescriptor[index].size = 1;
    }
}

// Update item comment
function updateItemComment(index) {
    const input = document.querySelector(`.report-item-comment[data-index="${index}"]`);
    currentDescriptor[index].comment = input.value;
}

// Delete report item
function deleteReportItem(index) {
    currentDescriptor.splice(index, 1);
    renderReportItems();
}

// Generate HID report descriptor bytes
function generateDescriptorBytes(descriptor) {
    const bytes = [];

    descriptor.forEach(item => {
        const tag = TAG_MAP[item.tag];
        const value = item.value || 0;
        const size = getItemDataSize(item);

        // Create item prefix byte
        let prefix = tag | size;
        bytes.push(prefix);

        // Add data bytes
        if (size === 1) {
            bytes.push(value & 0xFF);
        } else if (size === 2) {
            bytes.push(value & 0xFF);
            bytes.push((value >> 8) & 0xFF);
        } else if(size === 4) {
            bytes.push(value & 0xFF);
            bytes.push((value >> 8) & 0xFF);
            bytes.push((value >> 16) & 0xFF);
            bytes.push((value >> 24) & 0xFF);
        }
    });

    return bytes;
}

// Calculate report sizes
function calculateReportSizes(descriptor) {
    let inputSize = 0;
    let outputSize = 0;
    let featureSize = 0;
    let currentSize = 0;
    let currentCount = 1;
    let lastItemType = null;

    descriptor.forEach(item => {
        if (item.tag === 'REPORT_SIZE') {
            currentSize = item.value;
        } else if (item.tag === 'REPORT_COUNT') {
            currentCount = item.value;
        } else if (item.tag === 'INPUT') {
            inputSize += (currentSize * currentCount);
        } else if (item.tag === 'OUTPUT') {
            outputSize += (currentSize * currentCount);
        } else if (item.tag === 'FEATURE') {
            featureSize += (currentSize * currentCount);
        }
    });

    return {
        input: Math.ceil(inputSize / 8),
        output: Math.ceil(outputSize / 8),
        feature: Math.ceil(featureSize / 8)
    };
}

// Generate output
function generateOutput() {
    if (currentDescriptor.length === 0) {
        alert('No report descriptor items defined!');
        return;
    }

    const bytes = generateDescriptorBytes(currentDescriptor);
    const reportSizes = calculateReportSizes(currentDescriptor);

    // Update statistics
    document.getElementById('totalSize').textContent = `${bytes.length} bytes`;
    document.getElementById('itemCount').textContent = currentDescriptor.length;
    document.getElementById('inputReportSize').textContent = `${reportSizes.input} bytes`;
    document.getElementById('outputReportSize').textContent = `${reportSizes.output} bytes`;
    document.getElementById('featureReportSize').textContent = `${reportSizes.feature} bytes`;

    // Generate C array
    generateCArray(currentDescriptor);

    // Generate hex format
    generateHexFormat(bytes);

    // Generate human-readable format
    generateReadableFormat(currentDescriptor);
}

// Generate C array format
function generateCArray(descriptor) {
    const descriptorLength = generateDescriptorBytes(descriptor).length;
    const formatByte = (value) => `0x${(value & 0xFF).toString(16).toUpperCase().padStart(2, '0')}`;
    const formatItemBytes = (item) => {
        const tag = TAG_MAP[item.tag];
        const value = item.value || 0;
        const size = getItemDataSize(item);
        const bytes = [tag | size];

        if (size === 1) {
            bytes.push(value & 0xFF);
        } else if (size === 2) {
            bytes.push(value & 0xFF, (value >> 8) & 0xFF);
        } else if (size === 4) {
            bytes.push(value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF);
        }

        return bytes;
    };
    const formatItemComment = (item, depth) => {
        const indent = '  '.repeat(depth);
        if (!item.comment || item.tag === 'END_COLLECTION') {
            return `${indent}${item.tag}`;
        }

        return `${indent}${item.tag} (${item.comment})`;
    };

    let output = `/* USB HID Report descriptor Length */\n`;
    output += `#define UX_HID_REPORT_DESCRIPTOR_LENGTH ${descriptorLength}\n\n`;
    output += `/* USB HID Report descriptor */\n`;
    output += `unsigned char hid_report_descriptor[] = {\n`;

    let collectionDepth = 0;
    const renderedItems = descriptor.map((item, index) => {
        const itemBytes = formatItemBytes(item);
        const byteText = itemBytes.map(formatByte).join(', ');
        const withComma = `${byteText}${index < descriptor.length - 1 ? ',' : ''}`;
        return { item, withComma };
    });
    const commentColumnWidth = renderedItems.reduce((max, entry) => Math.max(max, entry.withComma.length), 0) + 2;

    renderedItems.forEach((entry) => {
        const { item, withComma } = entry;
        if (item.tag === 'END_COLLECTION') {
            collectionDepth = Math.max(0, collectionDepth - 1);
        }

        const paddedBytes = withComma.padEnd(commentColumnWidth, ' ');
        const itemComment = formatItemComment(item, collectionDepth);

        output += `    ${paddedBytes}// ${itemComment}\n`;

        if (item.tag === 'COLLECTION') {
            collectionDepth += 1;
        }
    });

    output += `};\n`;

    document.getElementById('outputC').textContent = output;
}

// Generate hex format
function generateHexFormat(bytes) {
    let output = '';

    for (let i = 0; i < bytes.length; i += 16) {
        const line = bytes.slice(i, i + 16)
            .map(b => b.toString(16).toUpperCase().padStart(2, '0'))
            .join(' ');
        output += `${i.toString(16).toUpperCase().padStart(4, '0')}: ${line}\n`;
    }

    document.getElementById('outputHex').textContent = output;
}

// Generate human-readable format
function generateReadableFormat(descriptor) {
    let output = '';
    let indent = 0;

    descriptor.forEach((item, index) => {
        // Adjust indentation
        if (item.tag === 'END_COLLECTION') {
            indent = Math.max(0, indent - 2);
        }

        const spaces = ' '.repeat(indent);
        const tagName = item.tag.replace(/_/g, ' ');
        const valueStr = `0x${item.value.toString(16).toUpperCase().padStart(item.size === 2 ? 4 : 2, '0')}`;
        const comment = item.comment ? ` // ${item.comment}` : '';

        output += `${spaces}${tagName} (${valueStr})${comment}\n`;

        // Increase indent after COLLECTION
        if (item.tag === 'COLLECTION') {
            indent += 2;
        }
    });

    document.getElementById('outputReadable').textContent = output;
}

// Copy to clipboard
function copyToClipboard(elementId, buttonElement) {
    const element = document.getElementById(elementId);
    const text = element.textContent;

    navigator.clipboard.writeText(text).then(() => {
        if (buttonElement) {
            const label = buttonElement.querySelector('.btn-label');
            const originalText = label ? label.textContent : buttonElement.textContent;
            if (label) {
                label.textContent = 'Copied!';
            } else {
                buttonElement.textContent = 'Copied!';
            }
            buttonElement.style.background = '#50fa7b';

            setTimeout(() => {
                if (label) {
                    label.textContent = originalText;
                } else {
                    buttonElement.textContent = originalText;
                }
                buttonElement.style.background = '#667eea';
            }, 2000);
            return;
        }

        alert('Copied to clipboard!');
    }).catch(err => {
        console.error('Failed to copy:', err);
    });
}

// Save descriptor as JSON
function saveJSON() {
    const data = {
        deviceType: document.getElementById('deviceType').value,
        descriptor: currentDescriptor,
        timestamp: new Date().toISOString()
    };

    const json = JSON.stringify(data, null, 2);
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);

    const a = document.createElement('a');
    a.href = url;
    a.download = 'hid_report_descriptor.json';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

// Load JSON file
function loadJSON() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';

    input.onchange = function(e) {
        const file = e.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = function(event) {
            try {
                const data = JSON.parse(event.target.result);

                if (data.descriptor) {
                    currentDescriptor = data.descriptor;
                    if (data.deviceType) {
                        document.getElementById('deviceType').value = data.deviceType;
                    }
                    renderReportItems();
                    alert('Descriptor loaded successfully!');
                } else {
                    alert('Invalid JSON format!');
                }
            } catch (err) {
                alert('Error parsing JSON: ' + err.message);
            }
        };
        reader.readAsText(file);
    };

    input.click();
}

// Load template
async function loadTemplate() {
    const loaded = await loadDeviceTemplate();
    if (loaded) {
        alert('Template loaded!');
    } else {
        alert('Template not found for selected device type.');
    }
}

// New descriptor
function newDescriptor() {
    if (currentDescriptor.length > 0) {
        if (!confirm('This will clear the current descriptor. Continue?')) {
            return;
        }
    }

    currentDescriptor = [];
    document.getElementById('deviceType').value = 'custom';
    renderReportItems();
}

// Update advanced settings
function updateAdvancedSettings() {
    const useReportId = document.getElementById('useReportId').checked;
    document.getElementById('reportIdCount').disabled = !useReportId;
}

// Toggle collapse
function toggleCollapse(id, button) {
    const element = document.getElementById(id);
    const isExpanded = element.classList.contains('show');

    if (isExpanded) {
        element.classList.remove('show');
        button.classList.add('collapsed');
        button.setAttribute('aria-expanded', 'false');
    } else {
        element.classList.add('show');
        button.classList.remove('collapsed');
        button.setAttribute('aria-expanded', 'true');
    }
}

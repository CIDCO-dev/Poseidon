const { Builder, By, Key, until } = require('selenium-webdriver');

// Webdriver declaration. change 'firefox' to 'chrome' to test on chrome browser
const driver = new Builder().forBrowser('firefox').build();

(async function AutomatedTest() {
    // Url to start the test on, modify whenever relevant.
    // Must begin test on login page, unless 'Login()' function is commented out or removed
    const baseURL = 'http://127.0.0.1';

    // Array containing test functions
    const pagesToTest = [
        { name: 'Dashboard', testFunction: Dashboard },
        { name: 'Map', testFunction: Map },
        { name: 'System Status', testFunction: Status },
        { name: 'Download Data', testFunction: Data },
        { name: 'Calibration', testFunction: Calibration },
        { name: 'Settings', testFunction: Settings }
    ];

    let errorLocation = 'Main';

    try {
        // Sometimes browser is slower to remove overlay causing problems, a retry should work
        await driver.get(baseURL);
        await wait(2000); // Wait for overlay

        // Function array loop. In case of error, user will be notified of location through the cmd
        for (const page of pagesToTest) {
            try {
                console.log(`Testing ${page.name} page.`);
                await page.testFunction(driver); // Pass the driver instance
            } catch (error) {
                errorLocation = page.name;
                throw error;
            }
        }

        console.log('Automated test successful.');
    } catch (error) {
        console.error(`Error occurred on ${errorLocation} page testing:`, error);
    } finally {
        // Quit the WebDriver instance after the test is done.
        console.log('Automated test terminated.');
        await driver.quit();
    }
})();

// Function for explicit wait to be used later
async function waitForElement(selector) {
    await driver.wait(until.elementLocated(selector), 10000);
    return driver.findElement(selector);
}

// function to wait out overlay
function wait(duration) {
    return new Promise((resolve) => setTimeout(resolve, duration));
}

async function checkClassPresence(elementId, className, shouldExist) {
    // Find the element with the specified class name
    const element = await driver.findElement(By.id(elementId));

    // Execute JavaScript to check for the class presence
    const hasClass = await driver.executeScript(
        'return arguments[0].classList.contains(arguments[1]);',
        element,
        className
    );

    // return an error if the class should be there and isnt or souldn't and is
    if ((shouldExist && !hasClass) || (!shouldExist && hasClass)) {
        throw new Error(`The class "${className}" is ${shouldExist ? 'absent' : 'present'} on the element.`);
    }
}

// Function to test side bar toggling
async function sidebarToggle() {
    pageTitle = await driver.getTitle();

    // Loop to click 'sidebarToggleTop' 4 times
    console.log('Testing sidebar toggles.');
    const sidebarToggleTop = await waitForElement(By.id('sidebarToggleTop'));
    await sidebarToggleTop.click();

    await wait(1000);

    // Check if the sidebar-toggled class is present
    await checkClassPresence('page-top', 'sidebar-toggled', true);

    await sidebarToggleTop.click(); // Double-clicking for demonstration

    await wait(1000);

    // Check if the sidebar-toggled class is absent
    await checkClassPresence('page-top', 'sidebar-toggled', false);

    // Loop to click 'sidebarToggle' 4 times
    const sidebarToggle = await waitForElement(By.id('sidebarToggle'));
    await sidebarToggle.click();

    await wait(1000);

    // Check if the sidebar-toggled class is present
    await checkClassPresence('page-top', 'sidebar-toggled', true);

    await sidebarToggle.click(); // Double-clicking for demonstration

    await wait(1000);

    // Check if the sidebar-toggled class is absent
    await checkClassPresence('page-top', 'sidebar-toggled', false);

    console.log('Sidebar toggles functional.')
}

// Function to test navigation between pages. 
async function pageNavigation() {
    console.log('Testing page navigation');
    pageTitle = await driver.getTitle();
    let errorNbr = 0;
    let errorLog = '';

    pageTitle = pageTitle.replace('Hydroball - ', '');
    if (pageTitle === 'Calibration' || pageTitle === 'Settings') {
        var link = await waitForElement(By.linkText('System Configuration'));
        await link.click();
        var link = await waitForElement(By.linkText(pageTitle));
        await link.click();
    } else {
        var link = await waitForElement(By.linkText(pageTitle));
        await link.click();
    }
    var title = await driver.getTitle();
    if (title === `Hydroball - ${pageTitle}`) {
        console.log(`${pageTitle} reached`);
    } else {
        console.log('Wrong page');
        errorLog += ` ${pageTitle},`;
        errorNbr += 1;
    }
    await wait(1000);

    if (pageTitle != 'Dashboard') {
        var link = await waitForElement(By.linkText('Dashboard'));
        await link.click();
        var title = await driver.getTitle();
        if (title === 'Hydroball - Dashboard') {
            console.log('Dashboard reached');
        } else {
            console.log('Wrong page');
            errorLog += ' Dashboard,';
            errorNbr += 1;
        }
        await wait(1000);
        driver.navigate().back();
    }

    if (pageTitle != 'Map') {
        link = await waitForElement(By.linkText('Map'));
        await link.click();
        await wait(1000);
        title = await driver.getTitle();
        if (title === 'Hydroball - Map') {
            console.log('Map reached');
        } else {
            console.log('Wrong page');
            errorLog += ' Map,';
            errorNbr += 1;
        }
        await driver.navigate().back();
        await wait(1000);
    }

    if (pageTitle != 'System Status') {
        link = await waitForElement(By.linkText('System Status'));
        await link.click();
        await wait(1000);
        title = await driver.getTitle();
        if (title === 'Hydroball - System Status') {
            console.log('System Status reached');
        } else {
            console.log('Wrong page');
            errorLog += ' System Status,';
            errorNbr += 1;
        }
        await driver.navigate().back();
        await wait(1000);
    }

    if (pageTitle != 'Download Data') {
        link = await waitForElement(By.linkText('Download Data'));
        await link.click();
        await wait(1000);
        title = await driver.getTitle();
        if (title === 'Hydroball - Download Data') {
            console.log('Download Data reached');
        } else {
            console.log('Wrong page');
            errorLog += ' Download Data,';
            errorNbr += 1;
        }
        await driver.navigate().back();
        await wait(1000);
    }

    if (pageTitle != 'Calibration') {
        link = await waitForElement(By.linkText('System Configuration'));
        await link.click();
        await wait(1000);
        link = await waitForElement(By.linkText('Calibration'));
        await link.click();
        await wait(1000);
        title = await driver.getTitle();
        if (title === 'Hydroball - Calibration') {
            console.log('Calibration reached');
        } else {
            console.log('Wrong page');
            errorLog += ' Calibration,';
            errorNbr += 1;
        }
        driver.navigate().back();
        await wait(1000);
    }

    if (pageTitle != 'Settings') {
        link = await waitForElement(By.linkText('System Configuration'));
        await link.click();
        await wait(1000);
        link = await waitForElement(By.linkText('Settings'));
        await link.click();
        await wait(1000);
        title = await driver.getTitle();
        if (title === 'Hydroball - Settings') {
            console.log('Settings reached');
        } else {
            console.log('Wrong page');
            errorLog += ' Settings,';
            errorNbr += 1;
        }
        await driver.navigate().back();
        await wait(1000);
    }

    // If any errors logged tell the tester
    if (errorNbr > 0) {
        throw new Error(`Errors logged in ${errorLog} page(s) in navigation from page ${pageTitle}.`);
    } else {
        console.log(`${pageTitle} navigation functional.`)
    }

}

async function Dashboard() {
    // Clicking on the 'sidebar-brand' element
    const sidebarBrand = await waitForElement(By.className('sidebar-brand'));
    await sidebarBrand.click();

    // Check if 'Dashboard - System Status' title is displayed
    const dashboardTitle = await driver.getTitle();
    if (dashboardTitle === 'Hydroball - Dashboard') {
        console.log('Dashboard reached');
    } else {
        throw new Error('Wrong page, we meant to reach Dashboard and got ', dashboardTitle);
    }

    //wait for overlay to leave
    await wait(1000);

    // Test recording buttons
    await testRecording(true);

    // Test sidebar toggles
    await sidebarToggle();

    // Test navigation to all pages from this page. 
    await pageNavigation();

    // if we make it here then it works
    console.log('Dashboard fully functional!');
}

async function testRecording(both) {
    const pageTitle = await driver.getTitle();
    console.log(`Testing recording button(s) on page ${pageTitle}.`);

    // check if we're testing 2 or 1 button depending on page
    if (both) {
        // Click, button then verify it has and is missing the appropriate classes
        const btnRecording = await waitForElement(By.id('btnRecording'));
        await btnRecording.click();
        await checkClassPresence('btnRecording', 'btn-success', false);
        await checkClassPresence('btnRecording', 'btn-danger', true);
        await checkClassPresence('RecIcon', 'text-success', false);
        await checkClassPresence('RecIcon', 'text-danger', true);

        await btnRecording.click();
        await checkClassPresence('btnRecording', 'btn-success', true);
        await checkClassPresence('btnRecording', 'btn-danger', false);
        await checkClassPresence('RecIcon', 'text-success', true);
        await checkClassPresence('RecIcon', 'text-danger', false);
    }
    // Click, button then verify it has and is missing the appropriate classes
    const recIcon = await waitForElement(By.id('RecIcon'));
    await recIcon.click();
    await checkClassPresence('RecIcon', 'text-success', false);
    await checkClassPresence('RecIcon', 'text-danger', true);
    if (both) {
        await checkClassPresence('btnRecording', 'btn-success', false);
        await checkClassPresence('btnRecording', 'btn-danger', true);
    }

    await recIcon.click();
    await checkClassPresence('RecIcon', 'text-success', true);
    await checkClassPresence('RecIcon', 'text-danger', false);
    if (both) {
        await checkClassPresence('btnRecording', 'btn-success', true);
        await checkClassPresence('btnRecording', 'btn-danger', false);
    }

    console.log(`Recording button(s) on page ${pageTitle} functional.`);
}

async function Map() {
    // Clicking on the 'Map' link
    var mapLink = await waitForElement(By.linkText('Map'));
    await mapLink.click();

    // Check if 'Hydroball - Map' title is displayed
    const mapTitle = await driver.getTitle();
    if (mapTitle === 'Hydroball - Map') {
        console.log('Map reached');
    } else {
        throw new Error('Wrong page, we meant to reach Dashboard and got ', mapTitle);
    }

    //wait for overlay to disappear
    await wait(1000);

    // Loop to click 'RecIcon' 4 times
    await testRecording();

    // Sidebar toggles testing
    await sidebarToggle();

    // Test navigation to all pages from this page. 
    await pageNavigation();

    // if we make it here then it works
    console.log('Map fully functional!');
}

async function Status() {
    // Clicking on the 'System Status' link
    var Link = await waitForElement(By.linkText('System Status'));
    await Link.click();

    // Check if 'Hydroball - System Status' title is displayed
    const systemStatusTitle = await driver.getTitle();
    if (systemStatusTitle === 'Hydroball - System Status') {
        console.log('System Status reached');
    } else {
        throw new Error('Wrong page, we meant to reach Dashboard and got ', systemStatusTitle);
    }

    //wait for overlay to leave
    await wait(1000);

    await sidebarToggle();

    // Test navigation to all pages from this page. 
    await pageNavigation();

    // if we make it here then it works
    console.log('System Status fully functional!');
}

async function Data() {
    // Wait for page to load
    await wait(3000)
    // Clicking on the Download data link
    var dataLink = await waitForElement(By.linkText('Download Data'));
    await dataLink.click();

    // Check if title is displayed 
    const dataTitle = await driver.getTitle();
    if (dataTitle === 'Hydroball - Download Data') {
        console.log('Download Data Reached');
    } else {
        throw new Error('Wrong page, we meant to reach Dashboard and got ', dataTitle);
    }

    //wait for overlay to leave
    await wait(1000);

    //Top of table buttons testing
    await Buttons();

    // Datatable testing
    await Table();

    // Sidebars toggles testing
    await sidebarToggle()

    // Test navigation to all pages from this page. 
    await pageNavigation();

    // if we make it here then it works
    console.log('Download Data fully functional!');
}

async function Buttons() {
    console.log('Testing table buttons.');
    // test select all button 
    const selectAll = await waitForElement(By.id('selectBtn'));
    for (let i = 0; i < 4; i++) {
        await selectAll.click();
        await wait(1000);
    }

    // test delete button 
    for (let i = 0; i < 2; i++) {
        const deleteBtn = await waitForElement(By.id('delBtn'));
        var checkboxOne = await waitForElement(By.className('custom-control custom-checkbox'));
        var yes = await waitForElement(By.id('yesDel'));
        var no = await waitForElement(By.id('noDel'));

        // Wait out last loop
        await wait(1000);
        checkboxOne = await waitForElement(By.className('custom-control custom-checkbox'));
        await checkboxOne.click();
        await deleteBtn.click();

        // Wait for modal to open
        await wait(1000);
        no = await waitForElement(By.id('noDel'));
        await no.click();

        // Wait for modal to close then click delete again
        await wait(1000);
        await deleteBtn.click();

        // Wait for modal to open
        await wait(1000);
        yes = await waitForElement(By.id('yesDel'));
        await yes.click();
    }
    console.log('Table buttons functional.');
}

async function Table() {
    console.log('Testing selector and table naviagtion');
    // test select and navigation and searchbar
    const selectElem = await waitForElement(By.id("perPageSelect"));
    const first = await waitForElement(By.linkText('First'));
    const last = await waitForElement(By.linkText('Last'));
    const prevBtn = await waitForElement(By.id('prevBtn'));
    const nextBtn = await waitForElement(By.id('nextBtn'));
    const searchBar = await waitForElement(By.name('searchbar'));

    for (let i = 0; i < 4; i++) {
        let j = 0;
        if (i === 0) {
            j = 5;
        } else if (i === 1) {
            j = 25;
        } else if (i === 2) {
            j = -1;
        } else if (i === 3) {
            j = 10;
        }
        console.log(`Clicking select ${j}`);
        // Clicking select element then wait for it to open
        await selectElem.click();
        await wait(1000);

        // Click the number represented by j then wait for it to close
        await selectElem.findElement(By.css(`option[value='${j}']`)).click();
        await wait(1000);


        // Pagination buttons testing and waiting between each
        console.log('Testing table navigation');
        await first.click();
        await wait(1000);
        await last.click();
        await wait(1000);
        await first.click();
        await wait(1000);
        if (j != -1) {
            const two = await waitForElement(By.id('page2'));
            await two.click();
            await wait(1000);
            const four = await waitForElement(By.id('page4'));
            await four.click();
            await wait(1000);
        }
        await nextBtn.click();
        await wait(1000);
        await nextBtn.click();
        await wait(1000);
        await first.click();
        await wait(1000);
        await prevBtn.click();
        await wait(1000);
        await prevBtn.click();
        await wait(1000);

        try {
            // Back to top
            if (i === 1 || i === 2) {
                const topPage = await waitForElement(By.className('scroll-to-top rounded'));
                await topPage.click();
                await wait(2000);
            }
        } catch {
            console.log('Scroll to top button unnecessary in this instance.')
        }

        //Searhcbar testing
        console.log('Testing searchbar')
        var searchItem = await waitForElement(By.className('tableElement'));
        searchItem = searchItem.getText();
        await searchBar.click();
        await searchBar.sendKeys(searchItem);
        await wait(3000);

        // Retrieve datatable and rows
        const dataTable = await driver.findElement(By.id('dataTable'));
        const rows = await dataTable.findElements(By.css('tr'));

        // Check the number of rows (remove the header row)
        if ((rows.length - 1) > 1) {
            throw new Error('The datatable has more than one row. The searchbar is broken');
        }

        await searchBar.clear();
        await searchBar.sendKeys(Key.RETURN);
        await wait(1000);
    }
}

async function Calibration() {
    // Wait for page to load
    await wait(1000)
    // Clicking on the System Configuration link
    var Link = await waitForElement(By.linkText('System Configuration'));
    await Link.click();
    await wait(1000);
    Link = await waitForElement(By.linkText('Calibration'));
    await Link.click();
    await wait(1000);

    // Check if title is displayed 
    const dataTitle = await driver.getTitle();
    console.log(dataTitle);
    if (dataTitle === 'Hydroball - Calibration') {
        console.log('Calibration Reached')
    } else {
        throw new Error('Wrong page, we meant to reach Dashboard and got ', dataTitle);
    }

    await sidebarToggle();

    // Test navigation to all pages from this page. 
    await pageNavigation();

    // if we make it here then it works
    console.log('Calibration fully functional!');
}

async function Settings() {
    // Wait for page to load
    await wait(1000)
    // Clicking on the System Configuration link
    Link = await waitForElement(By.linkText('System Configuration'));
    await Link.click();
    await wait(1000);
    Link = await waitForElement(By.linkText('Settings'));
    await Link.click();
    await wait(1000);

    // Check if title is displayed 
    const settingsTitle = await driver.getTitle();
    if (settingsTitle === 'Hydroball - Settings') {
        console.log('System Settings Reached')
    } else {
        throw new Error('Wrong page, we meant to reach Dashboard and got ', settingsTitle);
    }

    await sidebarToggle();

    // Test navigation to all pages from this page. 
    await pageNavigation();

    // if we make it here then it works
    console.log('Settings fully functional!');
}
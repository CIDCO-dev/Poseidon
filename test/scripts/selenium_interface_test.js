const { waitForDebugger } = require('inspector');
const { config } = require('process');
const { Builder, By, Key, until } = require('selenium-webdriver');
const driver = new Builder().forBrowser('chrome').build();

(async function AutomatedTest() {
    try {
        await driver.get('http://127.0.0.1');
        var errorLocation = 'Main'

        // Wait for overlay
        await wait(2000);

        try {
            console.log('Testing Dashboard page.');
            await Dashboard();
        } catch (error) {
            errorLocation = 'Dashboard';
            throw error;
        }
        try {
            console.log('Testing Map page.');
            await Map();
        } catch (error) {
            errorLocation = 'Map';
            throw error;
        }
        try {
            console.log('Testing System Status page.');
            await Status();
        } catch (error) {
            errorLocation = 'System Status';
            throw error;
        }
        try {
            console.log('Testing Download Data page.');
            await Data();
        } catch (error) {
            errorLocation = 'Download Data';
            throw error;
        }
        try {
            console.log('Testing Calibration page.');
            await Calibration();
        } catch (error) {
            errorLocation = 'Calibration';
            throw error;
        }
        try {
            console.log('Testing Settings page.');
            await Settings();
        } catch (error) {
            errorLocation = 'Settings';
            throw error;
        }
        console.log('Automated test successful.')
    } catch (error) {
        console.error(`Error occurred on ${errorLocation} page testing:`, error);
    } finally {
        // Quit the WebDriver instance after the test is done.
        console.log('Automated test terminated.')
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

// Function to test side bar toggling
async function sidebarToggle() {
    // Loop to click 'sidebarToggleTop' 4 times
    console.log('Testing sidebar toggles.');
    for (let i = 0; i < 4; i++) {
        const sidebarToggleTop = await waitForElement(By.id('sidebarToggleTop'));
        await sidebarToggleTop.click();
        await sidebarToggleTop.click(); // Double-clicking for demonstration
    }

    // Loop to click 'sidebarToggle' 4 times
    for (let i = 0; i < 4; i++) {
        const sidebarToggle = await waitForElement(By.id('sidebarToggle'));
        await sidebarToggle.click();
        await sidebarToggle.click(); // Double-clicking for demonstration
    }

    console.log('Sidebar toggles functional.')
}

// Function to test navigation between pages. 
async function pageNavigation() {
    try {
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
    } catch (error) {
        throw error;
    }
}

async function Dashboard() {
    // Clicking on the 'sidebar-brand' element
    const sidebarBrand = await waitForElement(By.className('sidebar-brand'));
    await sidebarBrand.click();

    // Check if 'Dashboard - System Status' title is displayed
    const dashboardTitle = await driver.getTitle();
    console.log(dashboardTitle);
    if (dashboardTitle === 'Hydroball - Dashboard') {
        console.log('Dashboard reached');
    } else {
        console.log('Wrong page');
    }

    //wait for overlay to buzz off
    await wait(1000);

    // Loop to click 'btnRecording' and 'RecIcon' 4 times
    for (let i = 0; i < 4; i++) {
        const btnRecording = await waitForElement(By.id('btnRecording'));
        await btnRecording.click();
        const recIcon = await waitForElement(By.id('RecIcon'));
        await recIcon.click();
    }

    await sidebarToggle();

    // Test navigation to all pages from this page. 
    await pageNavigation();

    // if we make it here then it works
    console.log('Dashboard fully functional!');
}

async function Map() {
    // Clicking on the 'Map' link
    var mapLink = await waitForElement(By.linkText('Map'));
    await mapLink.click();

    // Check if 'Hydroball - Map' title is displayed
    const mapTitle = await driver.getTitle();
    console.log(mapTitle);
    if (mapTitle === 'Hydroball - Map') {
        console.log('Map reached');
    } else {
        console.log('Wrong page');
    }

    //wait for overlay to disappear
    await wait(1000);

    // Loop to click 'RecIcon' 4 times
    for (let i = 0; i < 4; i++) {
        const recIcon = await waitForElement(By.id('RecIcon'));
        await recIcon.click();
        await wait(500)
        await recIcon.click(); // clicking twice for demonstration
    }

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
    console.log(systemStatusTitle);
    if (systemStatusTitle === 'Hydroball - System Status') {
        console.log('System Status reached');
    } else {
        console.log('Wrong page');
    }

    //wait for overlay to buzz off
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
    console.log(dataTitle);
    if (dataTitle === 'Hydroball - Download Data') {
        console.log('Download Data Reached')
    } else {
        console.log('Wrong page')
    }

    //wait for overlay to buzz off
    await wait(1000);

    // This function tests the select all and delete buttons
    async function Buttons() {
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
            await wait(1000)
            yes = await waitForElement(By.id('yesDel'));
            await yes.click();
        }
    }
    await Buttons();

    // This function tests the datatable navigation and the searchbar
    async function Table() {
        console.log('Testing selector and table naviagtion')
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
            console.log(`Clicking select ${j}`)
            // Clicking select element then wait for it to open
            await selectElem.click();
            await wait(1000);

            // Click the number represented by j then wait for it to close
            await selectElem.findElement(By.css(`option[value='${j}']`)).click();
            await wait(1000);


            // Pagination buttons testing and waiting between each
            console.log('Testing table navigation')
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
                // Back to top o// Test navigation to all pages from this page. f page
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
    await Table();

    await sidebarToggle()

    // Test navigation to all pages from this page. 
    await pageNavigation();

    // if we make it here then it works
    console.log('Download Data fully functional!');
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
        console.log('Wrong page')
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
    const dataTitle = await driver.getTitle();
    console.log(dataTitle);
    if (dataTitle === 'Hydroball - Settings') {
        console.log('System Settings Reached')
    } else {
        console.log('Wrong page')
    }

    await sidebarToggle();

    // Test navigation to all pages from this page. 
    await pageNavigation();

    // if we make it here then it works
    console.log('Settings fully functional!');
}
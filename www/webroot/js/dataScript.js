var i;
var dataln;
var fileslist = Array();
var linklist = Array();
var dataArray = [];
var olddataln = 0;
var state;

function processState(state) {
  //******************************
  //populating data model
  //******************************
  dataln = state.fileslist.length;
  fileslist = state.fileslist;
  //******************************
  //Creating links to files
  //******************************
  for (i = 0; i < dataln; i++) {
    linklist[i] =
      `<a class="tableElement" href="` +
      fileslist[i][1] +
      '" target="_blank" download>' +
      fileslist[i][0] +
      "</a>";
  }
  //******************************
  //populating display
  //******************************
  if (dataln !== olddataln) {
    olddataln = dataln;
    dataArray = [];
    for (i = 0; i < dataln; i++) {
      dataArray.push('<tr class="remove"><td><div class="custom-control custom-checkbox"><input type="checkbox" class="custom-control-input" id="' +
        fileslist[i][0] +
        '"><label class="custom-control-label" for="' +
        fileslist[i][0] +
        '"></label></div></td><td class="search">' +
        linklist[i] +
        "</td></tr>");
    }
    tableFilter(dataArray);
  }
}
//******************************
//Sends a delete command for selected file
//******************************
function sendDelete() {
  var rowsPerPage = parseInt($('#perPageSelect').val());
  rowsPerPage = (rowsPerPage === -1) ? 1 : rowsPerPage;
  for (i = 0; i < rowsPerPage; i++) {
    var checkBox = document.getElementById(fileslist[i][0]);
    if (checkBox.checked == true) {
      var cmd = { delete: fileslist[i][0] };
      socket.send(JSON.stringify(cmd));
      //socket.send("{\"delete\":[\""+ fileslist[i][0]+"\"]}");
    }
  }

}
//******************************
//Select all files
//******************************
function selectall() {
  var rowsPerPage = parseInt($('#perPageSelect').val());
  var anyBoxUnchecked = false;

  // Check if any box is unchecked
  for (i = 0; i < rowsPerPage; i++) {
    if (document.getElementById(fileslist[i][0]).checked === false) {
      anyBoxUnchecked = true;
      break;
    }
  }

  // Set the checked status based on the 'anyBoxUnchecked' flag
  for (i = 0; i < rowsPerPage; i++) {
    document.getElementById(fileslist[i][0]).checked = anyBoxUnchecked;
  }
}

//******************************
//Periodically poll for new files
//******************************
time = setInterval(function (list) {
  socket.send('{"f-list":"fileslist"}');
  setTimeout(list, 500);
}, 500);

//******************************
//Open websocket
//******************************

var socket = new WebSocket("ws://" + window.location.hostname + ":9003");
socket.onmessage = function (event) {
  // console.log(event.data);
  state = JSON.parse(event.data);
  processState(state);
};

//******************************
//Open websocket
//******************************

// Searchbar 
function Search(dataArray) {
  var value = $(".searchbar").val();
  value = value.toLowerCase();
  
  var filteredArray = dataArray.filter(function (item) {
    // Extract the part within the quotes of the id attribute then compare to search value
    var idValue = item.match(/id="([^"]+)"/);
    if (idValue) {
      idValue = idValue[1].toLowerCase();
      return idValue.includes(value);
    }
    // If idValue is not found, exclude the item from the filteredArray
    return false; 
  });

  return filteredArray;
}

// Search bar event handler
$(".searchbar").on("keyup", function () {
  var filteredData = Search(dataArray);
  tableFilter(filteredData);
});

// Searchbar display size depending on screen size
$(window).on("resize", function () {
  if ($(window).width() < 766) {
    $("#searchbarDiv").removeClass("ml-auto");
  } else { $("#searchbarDiv").addClass("ml-auto") }
});
if ($(window).width() < 766) {
  $("#searchbarDiv").removeClass("ml-auto");
} else { $("#searchbarDiv").addClass("ml-auto") }


//******************************
//Datatable filtering
//******************************

function tableFilter(dataArray) {
  var currentPage = 1;
  var rowsPerPage = parseInt($('#perPageSelect').val());
  var totalPages = Math.ceil(dataArray.length / rowsPerPage);
  if (rowsPerPage == -1) {
    totalPages = 1;
  }

  function displayTable() {
    var startIndex = (currentPage - 1) * rowsPerPage;
    var endIndex = startIndex + rowsPerPage;
    var displayedRows = dataArray.slice(startIndex, endIndex);

    // Clear existing rows
    $('#tableContents').empty();

    // Add rows to the table
    $.each(displayedRows, function (index, row) {
      $('#tableContents').append(row);
    });

    // Update pagination
    updatePagination();
  }

  function updatePagination() {
    var prevBtn = $('#prevBtn');
    var nextBtn = $('#nextBtn');
    var firstBtn = $('#firstBtn');
    var lastBtn = $('#lastBtn');

    // Disable/enable previous and next buttons based on current page
    prevBtn.toggleClass('disabled', currentPage === 1);
    firstBtn.toggleClass('disabled', currentPage === 1);
    nextBtn.toggleClass('disabled', currentPage === totalPages);
    lastBtn.toggleClass('disabled', currentPage === totalPages);

    // Clear existing pagination except for the previous and next buttons
    $('#pagination').find('.page-item:gt(1):lt(-2)').remove();

    // Calculate the maximum number of pages to display
    var maxPages = Math.min(totalPages, 5);

    if ($(window).width() < 766) {
      maxPages = Math.min(totalPages, 3); // Adjust the number of pages to display as desired for smaller screens
      firstBtn.text("<<");
      lastBtn.text(">>");
    }

    // Calculate the starting page based on the current page and maximum pages
    var startPage = Math.max(currentPage - Math.floor(maxPages / 2), 1);
    var endPage = Math.min(startPage + maxPages - 1, totalPages);

    // Generate pagination
    for (var i = startPage; i <= endPage; i++) {
      var pageItem = $(`<li class="page-item"><a id="page${i}" class="page-link pager" href="#"><span>${i}</span></a></li>`);
      if (i === currentPage) {
        pageItem.addClass('active');
      }
      pageItem.insertBefore(nextBtn.parent());
    }

    // Update page information
    var firstEntry = (currentPage - 1) * rowsPerPage + 1;
    var lastEntry = Math.min(currentPage * rowsPerPage, dataArray.length);
    var infoText = 'Showing ' + firstEntry + ' to ' + lastEntry + ' of ' + dataArray.length + ' entries';

    if (rowsPerPage === -1) {
      infoText = 'Showing ' + firstEntry + ' to ' + dataArray.length + ' of ' + dataArray.length + ' entries';
    }

    $('#dataTable_info').text(infoText);
  }

  // Handle perPageSelect change event
  $('#perPageSelect').change(function () {
    rowsPerPage = parseInt($(this).val());
    totalPages = Math.ceil(dataArray.length / rowsPerPage);
    currentPage = 1;
    displayTable();
  });

  // Handle pager click event
  $(document).on('click', '.pager', function (e) {
    e.preventDefault();
    currentPage = parseInt($(this).find('span').text(), 10);
    displayTable();
  });

  // Handle previous button click
  $('#prevBtn').click(function (e) {
    e.preventDefault();
    if (currentPage > 1) {
      currentPage--;
      displayTable();
    }
  });

  // Handle next button click
  $('#nextBtn').click(function (e) {
    e.preventDefault();
    if (currentPage < totalPages) {
      currentPage++;
      displayTable();
    }
  });

  // Handle first button click
  $('#firstBtn').click(function (e) {
    e.preventDefault();
    currentPage = 1;
    displayTable();
  });

  // Handle last button click
  $('#lastBtn').click(function (e) {
    e.preventDefault();
    currentPage = totalPages;
    displayTable();
  });

  // Initial table display
  displayTable();
};

// change nbr results per page event listener
$('#perPageSelect').on('change', function() {
  processState(state);
});


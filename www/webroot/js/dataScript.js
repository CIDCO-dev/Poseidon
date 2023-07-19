var i;
var dataln;
var fileslist = Array();
var linklist = Array();
var dataArray = [];
var olddataln = 0;

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
      '<a href="' +
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
    var table = document.getElementById("tableContents");
    var tabledata = " ";
    for (i = 0; i < dataln; i++) {
      if (tabledata.length < 2) {
        tabledata =
          '<tr class="remove"><td><div class="custom-control custom-checkbox"><input type="checkbox" class="custom-control-input" id="' +
          fileslist[i][0] +
          '"><label class="custom-control-label" for="' +
          fileslist[i][0] +
          '"></label></div></td><td class="search">' +
          linklist[i] +
          "</td></tr>";
        dataArray.push('<tr class="remove"><td><div class="custom-control custom-checkbox"><input type="checkbox" class="custom-control-input" id="' +
          fileslist[i][0] +
          '"><label class="custom-control-label" for="' +
          fileslist[i][0] +
          '"></label></div></td><td class="search">' +
          linklist[i] +
          "</td></tr>");
      } else {
        tabledata =
          tabledata +
          '<tr class="remove"><td><div class="custom-control custom-checkbox"><input type="checkbox" class="custom-control-input" id="' +
          fileslist[i][0] +
          '"><label class="custom-control-label" for="' +
          fileslist[i][0] +
          '"></label></div></td><td class="search">' +
          linklist[i] +
          "</td></tr>";
        dataArray.push('<tr class="remove"><td><div class="custom-control custom-checkbox"><input type="checkbox" class="custom-control-input" id="' +
          fileslist[i][0] +
          '"><label class="custom-control-label" for="' +
          fileslist[i][0] +
          '"></label></div></td><td class="search">' +
          linklist[i] +
          "</td></tr>");

      }
    }
    table.innerHTML = tabledata;
    tableFilter(dataArray);
  }
}
//******************************
//Sends a delete command for selected file
//******************************
function sendDelete() {
  for (i = 0; i < dataln; i++) {
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
  for (i = 0; i < dataln; i++) {
    document.getElementById(fileslist[i][0]).checked =
      !document.getElementById(fileslist[i][0]).checked;
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
  var state = JSON.parse(event.data);
  processState(state);
};

//******************************
//Open websocket
//******************************

// Searchbar 
function Search() {
  var value = $(".searchbar").val();
  value = value.toLowerCase();
  var files = $(".search");
  var el = $(".remove");
  for (i = 0; i < files.length; i++) {
    if (!files[i].innerHTML.toLowerCase().includes(value)) {
      el[i].style.display = "none";
    } else {
      el[i].style.display = "table-row";
    }
  }
}

// Ajustement de barre de recherche pour petit écrans
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
      var pageItem = $('<li class="page-item"><a class="page-link pager" href="#"><span>' + i + '</span></a></li>');
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



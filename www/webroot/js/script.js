(function ($) {
    "use strict"; // Start of use strict

    // Toggle the side navigation (3 horizontal bars)
    $("#sidebarToggle, #sidebarToggleTop").on('click', function (e) {
        if ($(".sidebar").hasClass("toggled") && $(window).width() > 768) {
            $('#homeIcon').css("display", "block");
        } else if ($(window).width() < 768) {
            $('#homeIcon').css("display", "block");
        } else {
            $('#homeIcon').css("display", "none");
        };
    });

    $("#sidebarToggle, #sidebarToggleTop").on('click', function (e) {
        if ($("#navWrap").hasClass("toggled")) {
            $("#navWrap").removeClass("mr-4");
        } else {
            $("#navWrap").addClass("mr-4");
        };
    });

    // Highlight active page in the sidebar
    $(".navbar-nav li").each(function() {
		var href = $(this).find('a').attr('href');
		if (window.location.pathname.includes(href)) {
			$(this).addClass('active');
		} else if (window.location.pathname.includes("calibration") || window.location.pathname.includes("settings")) {
            $("#utils").addClass('active');
        }
	});

    if ($(window).width() < 768) {
        $('#homeIcon').css("display", "block");
    }
    
})(typeof jQuery !== "undefined" ? jQuery : function(){ return { on: function(){}, hasClass: function(){}, css: function(){}, width: function(){ return 0; }, find: function(){ return { attr: function(){ return ''; } }; }, addClass: function(){}, removeClass: function(){}, each: function(){}, } }); // End of use strict

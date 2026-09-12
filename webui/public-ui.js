(function () {
  const themeKey = "site-theme";

  try {
    if (localStorage.getItem(themeKey) === "dark") {
      document.documentElement.classList.add("dark");
    }
  } catch (error) {
    // localStorage may be unavailable; the UI still works without persistence.
  }

  document.addEventListener("DOMContentLoaded", function () {
    if (document.body.classList.contains("public-shell")) return;

    const topNavigation = document.querySelector(".topnav");
    if (!topNavigation) return;

    const language = "__PUBLIC_UI_LANGUAGE__".slice(0, 2);
    const hasDigitalIo = __PUBLIC_UI_HAS_IO__;
    const labelsByLanguage = {
      de: {
        dashboard: "Dashboard",
        liveData: "Livedaten",
        bmsData: "BMS Daten",
        owTemperatures: "OW Temperaturen",
        settings: "Einstellungen",
        system: "System",
        interfaces: "Schnittstellen",
        serial: "Serial",
        bluetooth: "Bluetooth",
        digitalInputs: "Digitaleingänge",
        relays: "Relaisausgänge",
        onewire: "Onewire",
        onewireOffset: "Onewire II",
        dataMapping: "Datenquellen-Zuordnung",
        devices: "Geräte",
        neey: "NEEY Balancer",
        jbd: "JBD BMS",
        inverter: "Wechselrichter",
        general: "Allgemein",
        charge: "Laden",
        discharge: "Entladen",
        alarms: "Alarmregeln",
        temperature: "Temperatur",
        log: "Log",
        update: "Update",
        reboot: "Neustart",
        support: "Unterstützung",
        github: "BSC auf GitHub"
      },
      en: {
        dashboard: "Dashboard",
        liveData: "Live data",
        bmsData: "BMS data",
        owTemperatures: "OW temperatures",
        settings: "Settings",
        system: "System",
        interfaces: "Interfaces",
        serial: "Serial",
        bluetooth: "Bluetooth",
        digitalInputs: "Digital inputs",
        relays: "Relay outputs",
        onewire: "Onewire sensors",
        onewireOffset: "Onewire offsets",
        dataMapping: "Data device mapping",
        devices: "Devices",
        neey: "NEEY Balancer",
        jbd: "JBD BMS",
        inverter: "Inverter",
        general: "General",
        charge: "Charge",
        discharge: "Discharge",
        alarms: "Alarm rules",
        temperature: "Temperature",
        log: "Log",
        update: "Update",
        reboot: "Reboot",
        support: "Support",
        github: "BSC on GitHub"
      },
      nl: {
        dashboard: "Dashboard",
        liveData: "Live-gegevens",
        bmsData: "BMS gegevens",
        owTemperatures: "OW temperatuur",
        settings: "Instellingen",
        system: "Systeem",
        interfaces: "Interfaces",
        serial: "Serial",
        bluetooth: "Bluetooth",
        digitalInputs: "Digitale ingangen",
        relays: "Relaisuitgangen",
        onewire: "Onewire",
        onewireOffset: "Onewire offsets",
        dataMapping: "Toewijzing gegevensapparaten",
        devices: "Apparaten",
        neey: "NEEY Balancer",
        jbd: "JBD BMS",
        inverter: "Omvormers",
        general: "Algemeen",
        charge: "Laden",
        discharge: "Ontladen",
        alarms: "Alarmregels",
        temperature: "Temperatuur",
        log: "Log",
        update: "Update",
        reboot: "Reboot",
        support: "Support",
        github: "BSC op GitHub"
      }
    };
    const labels = labelsByLanguage[language] || labelsByLanguage.de;

    const navigation = [
      { type: "link", label: labels.dashboard, href: "/" },
      {
        type: "group",
        label: labels.liveData,
        prefix: "/livedata/",
        children: [
          { label: labels.bmsData, href: "/livedata/bscDataLive/" },
          { label: labels.owTemperatures, href: "/livedata/owTempLive/" }
        ]
      },
      { type: "section", label: labels.settings },
      { type: "link", label: labels.system, href: "/settings/system/" },
      {
        type: "group",
        label: labels.interfaces,
        prefix: "/settings/schnittstellen/",
        children: [
          { label: labels.serial, href: "/settings/schnittstellen/serial/" },
          /*{ label: labels.bluetooth, href: "/settings/schnittstellen/bt/" },*/
          { label: labels.digitalInputs, href: "/settings/schnittstellen/din/", requiresDigitalIo: true },
          { label: labels.relays, href: "/settings/schnittstellen/dout/", requiresDigitalIo: true },
          { label: labels.onewire, href: "/settings/schnittstellen/ow/" },
          { label: labels.onewireOffset, href: "/settings/schnittstellen/ow2/" },
          { label: labels.dataMapping, href: "/settings/schnittstellen/deviceMapping/" }
        ]
      },
      /*{
        type: "group",
        label: labels.devices,
        prefix: "/settings/devices/",
        children: [
          { label: labels.neey, href: "/settings/devices/neeyBalancer/" },
          { label: labels.jbd, href: "/settings/devices/jbdBms/" }
        ]
      },*/
      {
        type: "group",
        label: labels.inverter,
        prefix: "/settings/inverter/",
        children: [
          { label: labels.general, href: "/settings/inverter/bms_can/" },
          { label: labels.charge, href: "/settings/inverter/can_charge/" },
          { label: labels.discharge, href: "/settings/inverter/can_discharge/" }
        ]
      },
      {
        type: "group",
        label: labels.alarms,
        prefix: "/settings/alarm/",
        children: [
          { label: "BMS", href: "/settings/alarm/alarmBt/" },
          { label: labels.temperature, href: "/settings/alarm/alarmTemp/" }
        ]
      },
      { type: "section", label: "BSC" },
      { type: "link", label: labels.log, href: "/log/view/" },
      { type: "link", label: labels.update, href: "/settings/webota/" },
      { type: "action", label: labels.reboot, action: "reboot" },
      { type: "link", label: labels.support, href: "/support/" },
      {
        type: "link",
        label: labels.github,
        href: "https://github.com/shining-man/bsc_fw",
        external: true
      }
    ];

    function createLink(item) {
      const link = document.createElement("a");
      link.className = "public-nav-link";
      link.href = item.href;
      link.textContent = item.label;
      if (item.external) {
        link.target = "_blank";
        link.rel = "noopener noreferrer";
      }
      return link;
    }

    function createGroup(item, container) {
      const toggle = document.createElement("button");
      toggle.className = "public-nav-toggle";
      toggle.type = "button";
      toggle.dataset.prefix = item.prefix;
      toggle.setAttribute("aria-expanded", "false");

      const label = document.createElement("span");
      label.textContent = item.label;
      toggle.appendChild(label);

      const chevron = document.createElement("span");
      chevron.className = "public-nav-chevron";
      chevron.innerHTML = "&#8250;";
      toggle.appendChild(chevron);

      const subNavigation = document.createElement("div");
      subNavigation.className = "public-nav-sub";
      item.children.forEach(function (child) {
        if (child.requiresDigitalIo && !hasDigitalIo) return;
        subNavigation.appendChild(createLink(child));
      });

      container.appendChild(toggle);
      container.appendChild(subNavigation);
    }

    function buildSidebar() {
      const sidebar = document.createElement("nav");
      sidebar.className = "public-sidebar";
      sidebar.setAttribute("aria-label", "Navigation");

      const brand = document.createElement("a");
      brand.className = "public-side-brand";
      brand.href = "/";
      brand.innerHTML = "<img src='/favicon.svg' alt='BSC'>" +
        "<span class='public-brand-copy'><b>BSC</b>" +
        "<small>Battery Safety Controller</small></span>";
      sidebar.appendChild(brand);

      const navigationContainer = document.createElement("div");
      navigationContainer.className = "public-nav-scroll";

      navigation.forEach(function (item) {
        if (item.type === "link") {
          navigationContainer.appendChild(createLink(item));
        } else if (item.type === "group") {
          createGroup(item, navigationContainer);
        } else if (item.type === "section") {
          const section = document.createElement("div");
          section.className = "public-nav-section";
          section.textContent = item.label;
          navigationContainer.appendChild(section);
        } else if (item.type === "action") {
          const action = document.createElement("button");
          action.className = "public-nav-action";
          action.type = "button";
          action.dataset.action = item.action;
          action.textContent = item.label;
          navigationContainer.appendChild(action);
        }
      });

      sidebar.appendChild(navigationContainer);

      const footer = document.createElement("div");
      footer.className = "public-sidebar-foot";
      footer.innerHTML = "<button class='theme-toggle' type='button' aria-label='Theme'>" +
        "<span class='theme-icon'></span><span>Theme</span></button>";
      sidebar.appendChild(footer);

      return sidebar;
    }

    document.body.classList.add("public-shell");

    const menuButton = document.createElement("button");
    menuButton.className = "shell-menu";
    menuButton.type = "button";
    menuButton.setAttribute("aria-label", "Menu");
    menuButton.setAttribute("aria-expanded", "false");
    menuButton.innerHTML = "&#8801;";
    topNavigation.insertBefore(menuButton, topNavigation.firstChild);

    const overlay = document.createElement("div");
    overlay.className = "public-overlay";
    const sidebar = buildSidebar();
    document.body.insertBefore(overlay, document.body.firstChild);
    document.body.insertBefore(sidebar, overlay);

    const path = location.pathname;
    const links = sidebar.querySelectorAll("a.public-nav-link[href^='/']");
    let activeLink = null;

    links.forEach(function (link) {
      const href = link.getAttribute("href");
      const matchesRoot = href === "/" && path === "/";
      const matchesPath = href !== "/" && (path === href || path.indexOf(href) === 0);
      if (!matchesRoot && !matchesPath) return;
      if (!activeLink || href.length > activeLink.getAttribute("href").length) {
        activeLink = link;
      }
    });

    if (activeLink) activeLink.classList.add("active");

    const toggles = sidebar.querySelectorAll(".public-nav-toggle");

    function closeGroups() {
      toggles.forEach(function (toggle) {
        toggle.classList.remove("open");
        toggle.setAttribute("aria-expanded", "false");
        toggle.nextElementSibling.classList.remove("open");
      });
    }

    toggles.forEach(function (toggle) {
      const subNavigation = toggle.nextElementSibling;
      const containsActiveLink = activeLink && subNavigation.contains(activeLink);
      if (path.indexOf(toggle.dataset.prefix) === 0 || containsActiveLink) {
        toggle.classList.add("open");
        toggle.setAttribute("aria-expanded", "true");
        subNavigation.classList.add("open");
      }

      toggle.addEventListener("click", function () {
        const wasOpen = toggle.classList.contains("open");
        closeGroups();
        if (wasOpen) return;
        toggle.classList.add("open");
        toggle.setAttribute("aria-expanded", "true");
        subNavigation.classList.add("open");
      });
    });

    function closeSidebar() {
      sidebar.classList.remove("open");
      overlay.classList.remove("open");
      menuButton.setAttribute("aria-expanded", "false");
    }

    menuButton.addEventListener("click", function () {
      const open = !sidebar.classList.contains("open");
      sidebar.classList.toggle("open", open);
      overlay.classList.toggle("open", open);
      menuButton.setAttribute("aria-expanded", open ? "true" : "false");
    });

    overlay.addEventListener("click", closeSidebar);
    sidebar.addEventListener("click", function (event) {
      if (event.target.closest("a")) closeSidebar();
    });
    document.addEventListener("keydown", function (event) {
      if (event.key === "Escape") closeSidebar();
    });

    const rebootButton = sidebar.querySelector("[data-action='reboot']");
    rebootButton.addEventListener("click", function () {
      if (confirm("Reboot?")) location.href = "/restart/";
    });

    const themeButton = sidebar.querySelector(".theme-toggle");

    function updateThemeIcon() {
      themeButton.querySelector(".theme-icon").innerHTML =
        document.documentElement.classList.contains("dark") ? "&#9788;" : "&#9790;";
    }

    updateThemeIcon();
    themeButton.addEventListener("click", function () {
      const dark = document.documentElement.classList.toggle("dark");
      updateThemeIcon();
      try {
        localStorage.setItem(themeKey, dark ? "dark" : "light");
      } catch (error) {
        // Theme switching still works for the current page.
      }
    });
  });
})();

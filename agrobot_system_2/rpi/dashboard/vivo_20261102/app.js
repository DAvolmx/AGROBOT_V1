/* AgroBot · interfaz de la Raspberry Pi. Solo usa la API del mismo origen. */
(() => {
  "use strict";

  const $ = id => document.getElementById(id);
  const views = [...document.querySelectorAll(".view")];
  const nav = [...document.querySelectorAll(".nav button")];
  const number = new Intl.NumberFormat("es-CO", {maximumFractionDigits: 1});
  const COMMANDS = {
    EMERGENCY_STOP: {title:"Parada de emergencia", summary:"Solicita una parada de emergencia.", hex:"0x01", base64:"AQ=="},
    PARK: {title:"Aparcar", summary:"Solicita la maniobra de aparcamiento definida en el nodo.", hex:"0x02", base64:"Ag=="},
    PING: {title:"Ping", summary:"Comprueba el camino de control LoRaWAN.", hex:"0x03", base64:"Aw=="},
    ACK: {title:"ACK", summary:"Código de confirmación previsto en el backend.", hex:"0x04", base64:"BA=="}
  };
  const ALERT_LABELS = {
    plaga:"Plaga detectada", sensor_fuera_rango:"Sensor fuera de rango",
    sensor_caido:"Sensor sin dato", enlace_perdido:"Enlace LoRa perdido",
    downlink:"Downlink", sistema:"Avisos del sistema"
  };
  let selectedCommand = "EMERGENCY_STOP";
  let latestPacket = null;
  let history = [];
  let downlinks = [];
  let packetCount = null;
  let telegramState = null;
  let freshnessThreshold = 90;
  let socket = null;
  let retryTimer = null;
  let toastTimer = null;

  function toast(message) {
    const element = $("toast");
    element.textContent = message;
    element.classList.add("show");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => element.classList.remove("show"), 4300);
  }

  function showView(id) {
    views.forEach(view => view.classList.toggle("active", view.id === id));
    nav.forEach(button => button.classList.toggle("active", button.dataset.view === id));
    window.scrollTo({top:0, behavior:"smooth"});
  }

  function isInvalid(row, key) {
    if (!row) return true;
    const marks = [...(row._out_of_range || []), ...(row._sensors_missing || []), ...(row._ausentes || [])];
    return marks.includes(key) || row[key] === null || row[key] === undefined || !Number.isFinite(Number(row[key]));
  }

  function value(row, key, digits = 1) {
    if (isInvalid(row, key)) return "—";
    return new Intl.NumberFormat("es-CO", {minimumFractionDigits:digits, maximumFractionDigits:digits}).format(Number(row[key]));
  }

  function suspiciousSensorZeros(row) {
    return row && Number(row.tempSCD) === 0 && Number(row.humidity) === 0 &&
      ((row._out_of_range || []).includes("co2") || Number(row.co2) === 0);
  }

  function dateFromSeconds(seconds) {
    return Number.isFinite(Number(seconds)) && Number(seconds) > 0 ? new Date(Number(seconds) * 1000) : null;
  }

  function formatTime(seconds) {
    const date = dateFromSeconds(seconds);
    return date ? date.toLocaleString("es-CO", {dateStyle:"short", timeStyle:"medium"}) : "—";
  }

  function formatAge(seconds) {
    if (!Number.isFinite(seconds) || seconds < 0) return "—";
    if (seconds < 60) return `hace ${Math.round(seconds)} s`;
    if (seconds < 3600) return `hace ${Math.round(seconds / 60)} min`;
    if (seconds < 86400) return `hace ${Math.round(seconds / 3600)} h`;
    return `hace ${Math.round(seconds / 86400)} días`;
  }

  function setConnection(message, connected) {
    const badge = $("connection-badge");
    badge.textContent = message;
    badge.classList.toggle("demo", !connected);
    $("footer-status").textContent = message;
  }

  function refreshFreshness() {
    const banner = $("data-banner");
    if (!latestPacket || !latestPacket.received_at) {
      banner.className = "disclaimer";
      banner.replaceChildren();
      const bold = document.createElement("b"); bold.textContent = "Sin lecturas disponibles.";
      const description = document.createElement("span"); description.textContent = "El gateway aún no ha proporcionado telemetría.";
      banner.append(bold, description);
      $("quality-age").textContent = "—";
      return;
    }
    const age = Math.max(0, (Date.now() / 1000) - Number(latestPacket.received_at));
    const stale = age > freshnessThreshold;
    $("quality-age").textContent = formatAge(age);
    banner.className = stale ? "disclaimer error" : "disclaimer ok";
    banner.replaceChildren();
    const bold = document.createElement("b");
    bold.textContent = stale ? "Último dato obsoleto." : "Telemetría reciente.";
    const description = document.createElement("span");
    description.textContent = `Paquete recibido ${formatAge(age)} (${formatTime(latestPacket.received_at)}). Umbral de vigencia: ${freshnessThreshold} s.${stale ? " Los valores permanecen visibles como histórico, no como lectura en vivo." : ""}`;
    banner.append(bold, description);
    for (const id of ["meta-temp", "meta-humidity", "meta-co2"]) {
      const meta = $(id);
      if (meta.dataset.valid === "true") meta.textContent = meta.dataset.suspect === "true"
        ? `Valor 0 · revisar sensor${stale ? " · obsoleto" : ""}`
        : stale ? "Último valor · obsoleto" : "Último valor válido";
    }
  }

  function updatePacket(row) {
    if (!row || typeof row !== "object") return;
    latestPacket = row;
    for (const [key, valueId, metaId, unit, digits] of [
      ["tempSCD", "metric-temp", "meta-temp", "°C", 1],
      ["humidity", "metric-humidity", "meta-humidity", "%", 1],
      ["co2", "metric-co2", "meta-co2", "ppm", 0]
    ]) {
      const valid = !isInvalid(row, key);
      const valueElement = $(valueId);
      valueElement.replaceChildren();
      valueElement.append(document.createTextNode(`${valid ? value(row, key, digits) : "—"} `));
      const small = document.createElement("small"); small.textContent = unit;
      valueElement.append(small);
      const meta = $(metaId);
      meta.dataset.valid = String(valid);
      const suspect = valid && key !== "co2" && suspiciousSensorZeros(row);
      meta.dataset.suspect = String(suspect);
      meta.classList.toggle("warn", !valid || suspect);
      meta.textContent = !valid ? "Dato ausente o fuera de rango" : suspect ? "Valor 0 · revisar sensor" : "Último valor válido";
    }
    $("quality-rssi").textContent = Number.isFinite(Number(row.rssi)) ? `${number.format(Number(row.rssi))} dBm` : "—";
    $("quality-snr").textContent = Number.isFinite(Number(row.snr)) ? `${number.format(Number(row.snr))} dB` : "—";
    $("quality-sf").textContent = Number.isFinite(Number(row.sf)) ? `SF${row.sf}` : "—";
    refreshFreshness();
  }

  function setPacketCount(count) {
    if (!Number.isFinite(Number(count))) return;
    packetCount = Number(count);
    $("metric-packets").textContent = new Intl.NumberFormat("es-CO").format(packetCount);
    $("meta-packets").textContent = "Total conservado en la Pi";
  }

  function renderChart() {
    const values = history.slice().reverse().filter(row => !isInvalid(row, "tempSCD") && row.received_at).slice(-60);
    const trend = $("chart-trend"), area = $("chart-area");
    if (values.length < 2) {
      trend.setAttribute("d", ""); area.setAttribute("d", "");
      $("chart-period").textContent = "Se requieren al menos dos muestras válidas";
      return;
    }
    const temperatures = values.map(row => Number(row.tempSCD));
    let low = Math.min(...temperatures), high = Math.max(...temperatures);
    const margin = Math.max((high - low) * .15, .25);
    low -= margin; high += margin;
    const left = 42, right = 610, top = 25, bottom = 175;
    const points = temperatures.map((temp, index) => [
      left + index * (right - left) / (temperatures.length - 1),
      bottom - (temp - low) * (bottom - top) / (high - low)
    ]);
    const line = points.map(([x, y], index) => `${index ? "L" : "M"}${x.toFixed(1)} ${y.toFixed(1)}`).join(" ");
    trend.setAttribute("d", line);
    area.setAttribute("d", `${line} L${right} 186 L${left} 186 Z`);
    for (let i = 0; i < 4; i++) $("chart-label-" + i).textContent = `${number.format(high - i * (high - low) / 3)} °C`;
    const start = dateFromSeconds(values[0].received_at);
    const end = dateFromSeconds(values.at(-1).received_at);
    $("chart-label-4").textContent = start ? start.toLocaleTimeString("es-CO", {hour:"2-digit", minute:"2-digit"}) : "—";
    $("chart-label-5").textContent = "";
    $("chart-label-6").textContent = end ? end.toLocaleTimeString("es-CO", {hour:"2-digit", minute:"2-digit"}) : "—";
    $("chart-period").textContent = `${values.length} muestras válidas · ${formatTime(values[0].received_at)} a ${formatTime(values.at(-1).received_at)}`;
    if (temperatures.every(temp => temp === temperatures[0]))
      $("chart-period").textContent += " · serie constante; revisar sensor";
  }

  function renderHistory() {
    const body = $("history-body"); body.replaceChildren();
    if (!history.length) {
      const row = document.createElement("tr"), cell = document.createElement("td");
      cell.colSpan = 7; cell.textContent = "No hay paquetes guardados."; row.append(cell); body.append(row); return;
    }
    for (const packet of history.slice(0, 200)) {
      const row = document.createElement("tr");
      const invalid = [...(packet._out_of_range || []), ...(packet._sensors_missing || [])];
      const caution = suspiciousSensorZeros(packet) ? "temp/hum en cero" : "";
      const review = [...invalid, ...(caution ? [caution] : [])];
      const cells = [formatTime(packet.received_at), review.length ? "Revisar: " + review.join(", ") : "Válido",
        `${value(packet, "tempSCD", 1)} °C`, `${value(packet, "humidity", 1)} %`, `${value(packet, "co2", 0)} ppm`,
        Number.isFinite(Number(packet.rssi)) ? `${number.format(Number(packet.rssi))} dBm` : "—",
        Number.isFinite(Number(packet.snr)) ? `${number.format(Number(packet.snr))} dB` : "—"];
      cells.forEach((content, index) => {const cell = document.createElement("td"); if (index === 1) {const span = document.createElement("span");span.className = `status${review.length ? " warn" : ""}`;span.textContent = content;cell.append(span)} else cell.textContent = content;row.append(cell)});
      body.append(row);
    }
  }

  function addPacket(row) {
    const key = row.id ? `id:${row.id}` : `ts:${row.received_at}`;
    history = [row, ...history.filter(item => (item.id ? `id:${item.id}` : `ts:${item.received_at}`) !== key)].slice(0, 200);
    updatePacket(row); renderHistory(); renderChart();
  }

  function addDownlink(row) {
    const duplicate = downlinks.some(item => item.command === row.command && Math.abs(Number(item.sent_at) - Number(row.sent_at)) < 3);
    if (duplicate) return;
    downlinks.unshift(row);
    downlinks = downlinks.slice(0, 50);
    renderDownlinks();
  }

  function renderDownlinks() {
    const log = $("dl-log"); log.replaceChildren();
    if (!downlinks.length) {
      const row = document.createElement("div"); row.className = "dl-log-row";
      const message = document.createElement("strong"); message.textContent = "No hay downlinks registrados.";
      row.append(message); log.append(row); return;
    }
    for (const item of downlinks.slice(0, 30)) {
      const definition = COMMANDS[item.command];
      const state = item.ok === null || item.ok === undefined ? "Resultado antiguo no guardado" : item.ok ? "Aceptado por ChirpStack" : "Rechazado / error";
      const row = document.createElement("div"); row.className = "dl-log-row";
      const timeCell = document.createElement("div"); const time = document.createElement("strong");
      const date = dateFromSeconds(item.sent_at);
      time.textContent = date ? date.toLocaleTimeString("es-CO", {hour:"2-digit", minute:"2-digit"}) : "—";
      const day = document.createElement("small"); day.textContent = date ? date.toLocaleDateString("es-CO") : "";timeCell.append(time, day);
      const commandCell = document.createElement("div"); const command = document.createElement("strong");command.textContent = item.command || "—";
      const payload = document.createElement("small");payload.textContent = definition ? `Payload ${definition.hex} · FPort 10` : "Código desconocido";commandCell.append(command, payload);
      const stateCell = document.createElement("div");const tag = document.createElement("span");
      tag.className = `dl-stage${item.ok === null || item.ok === undefined ? " unknown" : item.ok ? "" : " error"}`;
      tag.textContent = state;stateCell.append(tag);
      const evidenceCell = document.createElement("div");const note = document.createElement("small");note.textContent = "Ejecución en el nodo no verificada";evidenceCell.append(note);
      row.append(timeCell, commandCell, stateCell, evidenceCell);log.append(row);
    }
  }

  async function loadJson(url, options) {
    const response = await fetch(url, options);
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  }

  async function loadInitial() {
    const results = await Promise.allSettled([
      loadJson("/api/estado"), loadJson("/api/history?limit=60"), loadJson("/api/downlinks?limit=30"), loadJson("/api/telegram")
    ]);
    if (results[0].status === "fulfilled") {
      const state = results[0].value;
      if (state.umbral_seg) freshnessThreshold = Number(state.umbral_seg);
      if (state.last_packet && Object.keys(state.last_packet).length) updatePacket(state.last_packet);
      setPacketCount(state.packet_count);
    }
    if (results[1].status === "fulfilled") {history = results[1].value;renderHistory();renderChart()}
    if (results[2].status === "fulfilled") {downlinks = results[2].value;renderDownlinks()}
    if (results[3].status === "fulfilled") renderTelegram(results[3].value);
    if (results[0].status === "rejected") {setConnection("Gateway sin respuesta", false);toast("No fue posible consultar el gateway de la Pi.")}
    refreshFreshness();
  }

  function connectSocket() {
    clearTimeout(retryTimer);
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    socket = new WebSocket(`${protocol}//${location.host}/ws`);
    socket.onopen = () => setConnection("Datos en vivo", true);
    socket.onclose = () => {setConnection("Reconectando…", false);retryTimer = setTimeout(connectSocket, 3000)};
    socket.onerror = () => socket.close();
    socket.onmessage = event => {
      let message;
      try {message = JSON.parse(event.data)} catch {return}
      if (message.type === "snapshot") {
        const data = message.data || {};
        if (data.estado?.umbral_seg) freshnessThreshold = Number(data.estado.umbral_seg);
        history = data.history || history;
        downlinks = data.downlinks || downlinks;
        renderHistory();renderChart();renderDownlinks();
        if (data.last_packet && Object.keys(data.last_packet).length) updatePacket(data.last_packet);
        setPacketCount(data.packet_count);
      } else if (message.type === "packet") {
        addPacket(message.data);setPacketCount(message.packet_count);
      } else if (message.type === "downlink_sent") {
        addDownlink(message.data);
        toast(`${message.data.command}: ${message.data.ok ? "aceptado por ChirpStack" : "no aceptado"}. Ejecución no verificada.`);
        refreshTelegram();
      } else if (message.type === "alerta_externa") {
        refreshTelegram();
      }
    };
  }

  function selectCommand(command) {
    const definition = COMMANDS[command]; if (!definition) return;
    selectedCommand = command;
    document.querySelectorAll("[data-command]").forEach(button => {const active = button.dataset.command === command;button.classList.toggle("active", active);button.setAttribute("aria-pressed", String(active))});
    $("dl-title").textContent = definition.title;
    $("dl-summary").textContent = definition.summary;
    $("dl-command").textContent = command;
    $("dl-hex").textContent = definition.hex;
    $("dl-base64").textContent = definition.base64;
    $("dl-json").textContent = JSON.stringify({command});
  }

  async function sendDownlink() {
    const command = selectedCommand;
    const title = COMMANDS[command].title;
    if (!window.confirm(`¿Enviar el downlink ${title} (${command}) al nodo LoRaWAN?\n\nChirpStack puede aceptar la solicitud sin confirmar la ejecución en el robot.`)) return;
    const button = $("dl-prepare");button.disabled = true;button.textContent = "Enviando…";
    try {
      const result = await loadJson("/api/downlink", {method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({command})});
      addDownlink({command, ok:result.ok, sent_at:Date.now()/1000});
      toast(result.ok ? `${title}: aceptado por ChirpStack. Ejecución no verificada.` : `${title}: la Pi informó que no pudo encolarlo.`);
    } catch (error) {toast(`No se pudo enviar ${title}: ${error.message}`)}
    finally {button.disabled = false;button.textContent = "Enviar downlink"}
  }

  function renderTelegram(data) {
    telegramState = data;
    const active = Boolean(data.enabled && data.configurado);
    $("telegram-status").textContent = active ? "Servicio configurado y habilitado." : data.tiene_token ? "Bot configurado, pero deshabilitado o sin chat vinculado." : "Token del bot no configurado.";
    $("telegram-badge").textContent = active ? "Habilitado" : "Requiere atención";
    $("telegram-token").textContent = data.tiene_token ? "Configurado" : "No configurado";
    $("telegram-chat").textContent = data.chat_id ? "Sí" : "No · envía /start al bot";
    $("telegram-error").textContent = data.last_error || "Ninguno informado";
    const alerts = $("telegram-alerts");alerts.replaceChildren();
    for (const [key, enabled] of Object.entries(data.alerts || {})) {
      const label = document.createElement("label");label.className = "telegram-alert-row";
      const text = document.createElement("span");text.textContent = ALERT_LABELS[key] || key;
      const input = document.createElement("input");input.type = "checkbox";input.checked = Boolean(enabled);input.dataset.alert = key;
      label.append(text, input);alerts.append(label);
    }
    const log = $("telegram-log");log.replaceChildren();
    if (!data.log?.length) {log.textContent = "Sin mensajes registrados.";return}
    for (const entry of data.log.slice(0, 30)) {
      const item = document.createElement("div");item.className = "telegram-entry";
      const title = document.createElement("strong");title.textContent = `${entry.ok ? "Enviado" : "Error"} · ${entry.tipo || "Notificación"}`;
      const detail = document.createElement("small");detail.textContent = `${formatTime(entry.ts)} · ${(entry.texto || "").replace(/<[^>]*>/g, "").slice(0, 130)}`;
      item.append(title, detail);log.append(item);
    }
  }

  async function refreshTelegram() {
    try {renderTelegram(await loadJson("/api/telegram"))}
    catch {$("telegram-status").textContent = "No se pudo consultar Telegram."}
  }

  async function telegramAction(action) {
    if (!telegramState && action === "toggle") return;
    const urls = {test:"/api/telegram/test",report:"/api/telegram/report",toggle:"/api/telegram"};
    const options = {method:"POST"};
    if (action === "toggle") {options.headers = {"Content-Type":"application/json"};options.body = JSON.stringify({enabled:!telegramState.enabled})}
    try {
      const result = await loadJson(urls[action], options);
      toast(action === "toggle" ? "Configuración de Telegram actualizada." : result.ok ? "Telegram informó envío correcto." : `Telegram: ${result.error || "no pudo enviar"}`);
      await refreshTelegram();
    } catch (error) {toast(`Telegram: ${error.message}`)}
  }

  document.addEventListener("click", event => {
    const view = event.target.closest("[data-view]"); if (view) {showView(view.dataset.view);return}
    const command = event.target.closest("[data-command]"); if (command) {selectCommand(command.dataset.command);return}
    if (event.target.closest("#dl-prepare")) {sendDownlink();return}
    const telegram = event.target.closest("[data-telegram-action]"); if (telegram) telegramAction(telegram.dataset.telegramAction);
  });
  document.addEventListener("change", async event => {
    const input = event.target.closest("[data-alert]"); if (!input || !telegramState) return;
    const previous = !input.checked;
    try {
      await loadJson("/api/telegram", {method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({alerts:{...telegramState.alerts,[input.dataset.alert]:input.checked}})});
      await refreshTelegram();toast("Categoría de alerta actualizada.");
    } catch (error) {input.checked = previous;toast(`No se pudo guardar la alerta: ${error.message}`)}
  });

  renderHistory();renderDownlinks();refreshFreshness();
  loadInitial();connectSocket();
  setInterval(refreshFreshness, 1000);
  setInterval(refreshTelegram, 20000);
  setInterval(async () => {
    if (socket?.readyState === WebSocket.OPEN) return;
    try {const state = await loadJson("/api/estado");if (state.umbral_seg) freshnessThreshold = Number(state.umbral_seg);if (state.last_packet && Object.keys(state.last_packet).length) updatePacket(state.last_packet);setPacketCount(state.packet_count)} catch {}
  }, 15000);
})();

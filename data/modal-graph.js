/* =====================================
modal-graph.js - Versione con WebSocket condiviso
===================================== */

class EventGraphModal {
    constructor() {
        this.modal = document.getElementById('eventModal');
        this.chart = null;
        this.currentEvent = null;
        this.currentEventIndex = -1;
        this.events = []; // Lista degli eventi dalla tabella
        this.rawData = null;
        this.metadata = null;
        
        this.initializeModal();
    }
    
    initializeModal() {
        // Close handlers
        document.getElementById('closeModal').onclick = () => this.close();
        window.onclick = (event) => {
            if (event.target === this.modal) this.close();
        };
        
        // Control handlers
        document.getElementById('prevEvent').onclick = () => this.navigateEvent(-1);
        document.getElementById('nextEvent').onclick = () => this.navigateEvent(1);
        document.getElementById('durationSelect').onchange = () => this.refreshData();
        document.getElementById('showDetails').onchange = () => this.updateChart();
        
        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            if (this.modal.style.display === 'block') {
                if (e.key === 'Escape') this.close();
                if (e.key === 'ArrowLeft') this.navigateEvent(-1);
                if (e.key === 'ArrowRight') this.navigateEvent(1);
            }
        });
    }
    
    open(eventElement, eventsList) {
        // Estrai dati evento dal DOM
        const timestamp = eventElement.dataset.timestamp;
        const eventType = eventElement.cells[1].textContent;
        const eventName = eventElement.cells[2].textContent;
        
        this.currentEvent = {
            timestamp: parseInt(timestamp),
            type: eventType,
            name: eventName,
            element: eventElement
        };
        
        this.events = eventsList || [];
        this.currentEventIndex = this.events.indexOf(eventElement);
        
        // Aggiorna UI
        document.getElementById('modalTitle').textContent = 
            `${eventType} - ${eventName} (${new Date(this.currentEvent.timestamp * 1000).toLocaleString()})`;
        
        this.updateNavigationButtons();
        this.modal.style.display = 'block';
        
        // Carica dati
        this.refreshData();
    }
    
    close() {
        this.modal.style.display = 'none';
        if (this.chart) {
            this.chart.destroy();
            this.chart = null;
        }
        // Reset dati
        this.rawData = null;
        this.metadata = null;
        this.processedData = null;
    }
    
    navigateEvent(direction) {
        const newIndex = this.currentEventIndex + direction;
        if (newIndex >= 0 && newIndex < this.events.length) {
            this.open(this.events[newIndex], this.events);
        }
    }
    
    updateNavigationButtons() {
        document.getElementById('prevEvent').disabled = this.currentEventIndex <= 0;
        document.getElementById('nextEvent').disabled = this.currentEventIndex >= this.events.length - 1;
    }
    
    refreshData() {
        if (!this.currentEvent) return;
        
        this.showLoading(true);
        const duration = document.getElementById('durationSelect').value;
        
        // Reset dati precedenti
        this.rawData = null;
        this.metadata = null;
        this.processedData = null;
        
        // USA il WebSocket globale invece di creare uno nuovo
        const command = `get_buffer_window:${this.currentEvent.timestamp}:${duration}`;
        if (window.ws && window.ws.readyState === WebSocket.OPEN) {
            window.ws.send(command);
            this.showStatus(`Richiesta dati: ${duration}s attorno all'evento...`);
        } else {
            this.showStatus('WebSocket non connesso', true);
            this.showLoading(false);
        }
    }
    
    // Questa funzione viene chiamata dal message handler globale di index.html
    handleBinaryData(blob) {
        const reader = new FileReader();
        reader.onload = (e) => {
            const arrayBuffer = e.target.result;
            this.rawData = new Uint32Array(arrayBuffer);
            console.log(`Ricevuti ${this.rawData.length} campioni binari`);
            this.checkDataComplete();
        };
        reader.readAsArrayBuffer(blob);
    }
    
    // Questa funzione viene chiamata dal message handler globale di index.html
    handleMetadata(metadata) {
        this.metadata = metadata;
        console.log('Metadati ricevuti:', metadata);
        const durationLabel = metadata.duration_seconds ? `${metadata.duration_seconds}s` : 'N/A';
        this.showStatus(`Dati ricevuti: ${metadata.sample_count} campioni (${durationLabel})`);
        this.checkDataComplete();
    }
    
    checkDataComplete() {
        if (this.rawData && this.metadata) {
            this.processData();
            this.updateChart();
            this.showLoading(false);
        }
    }
    
    processData() {
        if (!this.rawData || !this.metadata) return;
        
        const samples = [];
        
        // Ogni campione è 32 bit packed secondo la struttura EncoderDataExtended
        for (let i = 0; i < this.rawData.length; i++) {
            const packed = this.rawData[i];
            
            // Estrai campi bit-packed
            const infrared = (packed >> 0) & 0x1;
            const detect = (packed >> 1) & 0x1;
            const door_open = (packed >> 2) & 0x1;
            const newcode = (packed >> 3) & 0x1;
            const temp_closed = (packed >> 4) & 0x1;
            const manual_open = (packed >> 5) & 0x1;
            const authorized = (packed >> 6) & 0x1;
            const servo_moving = (packed >> 7) & 0x1;
            const rawAngle = (packed >> 8) & 0xFFF;  // 12 bit
            const event_duration = (packed >> 20) & 0xFF; // 8 bit
            
            // Converti angolo in gradi (0-4095 -> 0-360°)
            const angleDegrees = (rawAngle / 4095) * 360;
            
            // Tempo in secondi dall'inizio della finestra
            const timeSeconds = i * 0.1; // Ogni campione = 100ms = 0.1s
            
            samples.push({
                time: timeSeconds,
                angle: angleDegrees,
                infrared: infrared,
                detect: detect,
                door_open: door_open,
                newcode: newcode,
                temp_closed: temp_closed,
                manual_open: manual_open,
                authorized: authorized,
                servo_moving: servo_moving,
                event_duration: event_duration / 10.0 // Converti in secondi
            });
        }
        
        this.processedData = samples;
        console.log(`Processati ${samples.length} campioni`);
    }
    
    updateChart() {
        if (!this.processedData) return;
        
        const ctx = document.getElementById('eventChart').getContext('2d');
        const showDetails = document.getElementById('showDetails').checked;
        
        // Assicurati che il chart precedente sia completamente distrutto
        if (this.chart) {
            this.chart.destroy();
            this.chart = null;
        }
        
        // Prepara datasets
        const datasets = [
            {
                label: 'Angolo Porta (°)',
                data: this.processedData.map(sample => ({
                    x: sample.time,
                    y: sample.angle
                })),
                borderColor: '#3498db',
                backgroundColor: 'rgba(52, 152, 219, 0.1)',
                borderWidth: 2,
                fill: false,
                yAxisID: 'angleAxis',
                pointRadius: 0,
                pointHoverRadius: 4
            }
        ];
        
        if (showDetails) {
            // Segnali digitali compatti con ampiezze ridotte e senza riempimento
            datasets.push(
                {
                    label: 'Infrarosso',
                    data: this.processedData.map(sample => ({
                        x: sample.time,
                        y: sample.infrared ? 5 : 0
                    })),
                    borderColor: '#e74c3c',
                    backgroundColor: 'rgba(231, 76, 60, 0.2)',
                    borderWidth: 1.5,
                    fill: false,
                    yAxisID: 'digitalAxis',
                    stepped: true,
                    pointRadius: 0
                },
                {
                    label: 'RFID Detect',
                    data: this.processedData.map(sample => ({
                        x: sample.time,
                        y: sample.detect ? 15 : 10
                    })),
                    borderColor: '#27ae60',
                    backgroundColor: 'rgba(39, 174, 96, 0.2)',
                    borderWidth: 1.5,
                    fill: false,
                    yAxisID: 'digitalAxis',
                    stepped: true,
                    pointRadius: 0
                },
                {
                    label: 'Porta Aperta',
                    data: this.processedData.map(sample => ({
                        x: sample.time,
                        y: sample.door_open ? 25 : 20
                    })),
                    borderColor: '#9b59b6',
                    backgroundColor: 'rgba(155, 89, 182, 0.2)',
                    borderWidth: 1.5,
                    fill: false,
                    yAxisID: 'digitalAxis',
                    stepped: true,
                    pointRadius: 0
                },
                {
                    label: 'New Code',
                    data: this.processedData.map(sample => ({
                        x: sample.time,
                        y: sample.newcode ? 35 : 30
                    })),
                    borderColor: '#f39c12',
                    backgroundColor: 'rgba(243, 156, 18, 0.2)',
                    borderWidth: 1.5,
                    fill: false,
                    yAxisID: 'digitalAxis',
                    stepped: true,
                    pointRadius: 0
                }
            );
            
            // Aggiungi temp_closed se presente
            if (this.processedData.some(s => s.temp_closed)) {
                datasets.push({
                    label: 'Temp Chiuso',
                    data: this.processedData.map(sample => ({
                        x: sample.time,
                        y: sample.temp_closed ? 45 : 40
                    })),
                    borderColor: '#e67e22',
                    backgroundColor: 'rgba(230, 126, 34, 0.2)',
                    borderWidth: 1.5,
                    fill: false,
                    yAxisID: 'digitalAxis',
                    stepped: true,
                    pointRadius: 0
                });
            }
        }
        
        // Configurazione Chart.js
        this.chart = new Chart(ctx, {
            type: 'line',
            data: { datasets },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    intersect: false,
                    mode: 'nearest'
                },
                plugins: {
                    title: {
                        display: true,
                        text: `Evento: ${this.currentEvent.type} - ${this.currentEvent.name}`,
                        font: { size: 16 }
                    },
                    legend: {
                        display: true,
                        position: 'top',
                        labels: {
                            boxWidth: 12,
                            padding: 15,
                            font: { size: 11 }
                        }
                    }
                },
                scales: {
                    x: {
                        type: 'linear',
                        title: {
                            display: true,
                            text: 'Tempo (secondi)'
                        },
                        grid: {
                            color: 'rgba(0,0,0,0.1)'
                        }
                    },
                    angleAxis: {
                        type: 'linear',
                        position: 'left',
                        title: {
                            display: true,
                            text: 'Angolo Porta (°)'
                        },
                        min: 0,
                        max: 360,
                        grid: {
                            color: 'rgba(52, 152, 219, 0.2)'
                        },
                        ticks: {
                            color: '#3498db'
                        }
                    }
                }
            }
        });
        
        // Aggiungi asse digitale se necessario
        if (showDetails) {
            this.chart.options.scales.digitalAxis = {
                type: 'linear',
                position: 'right',
                title: {
                    display: true,
                    text: 'Segnali Digitali'
                },
                min: 0,
                max: 360, // STESSA SCALA dell'asse angolo per renderli proporzionalmente piccoli
                grid: {
                    display: false
                },
                ticks: {
                    color: '#7f8c8d',
                    callback: function(value) {
                        const labels = {
                            '2.5': 'IR',
                            '12.5': 'RFID',
                            '22.5': 'Door',
                            '32.5': 'New',
                            '42.5': 'Temp'
                        };
                        return labels[value] || '';
                    }
                }
            };
            this.chart.update();
        }
        
        this.addEventMarkers();
    }
    
    addEventMarkers() {
        // Calcola quando dovrebbe essere l'evento nella timeline
        // Se timestamp_center è nel mezzo, e iniziamo 1 secondo prima...
        const eventTimeInGraph = 1.0; // 1 secondo dall'inizio del grafico
        
        console.log(`Evento dovrebbe essere a t=${eventTimeInGraph}s nel grafico`);
    }
    
    showLoading(show) {
        document.getElementById('loadingOverlay').style.display = show ? 'flex' : 'none';
    }
    
    showStatus(message, isError = false) {
        const statusElement = document.getElementById('statusInfo');
        statusElement.textContent = message;
        statusElement.style.color = isError ? '#e74c3c' : '#666';
    }
}

// Inizializza il modal quando il DOM è pronto
document.addEventListener('DOMContentLoaded', function() {
    window.eventGraphModal = new EventGraphModal();
});

// Funzione globale per aprire il modal (chiamata da index.html)
function openEventGraph(eventElement) {
    // Raccogli tutti gli eventi dalla tabella per navigazione
    const eventRows = Array.from(document.querySelectorAll('#logBody tr'));
    window.eventGraphModal.open(eventElement, eventRows);
}
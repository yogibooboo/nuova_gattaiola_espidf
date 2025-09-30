/* =====================================
modal-graph.js - Versione con legenda adaptive e dettagli sempre visibili
===================================== */

class EventGraphModal {
    constructor() {
        this.modal = document.getElementById('eventModal');
        this.chart = null;
        this.currentEvent = null;
        this.currentEventIndex = -1;
        this.events = [];
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

        // Control handlers - NAVIGAZIONE CORRETTA
        document.getElementById('prevEvent').onclick = () => this.navigateEvent(1);
        document.getElementById('nextEvent').onclick = () => this.navigateEvent(-1);

        // Gestione bottoni durata
        document.querySelectorAll('.btn-duration').forEach(btn => {
            btn.onclick = () => {
                document.querySelectorAll('.btn-duration').forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                this.refreshData();
            };
        });

        // Gestione toggle legenda
        this.legendVisible = false;
        document.getElementById('toggleLegend').onclick = () => {
            this.legendVisible = !this.legendVisible;
            document.getElementById('toggleLegend').textContent =
                this.legendVisible ? 'Nascondi Legenda' : 'Mostra Legenda';
            this.updateChart();
        };
        
        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            if (this.modal.style.display === 'block') {
                if (e.key === 'Escape') this.close();
                if (e.key === 'ArrowLeft') this.navigateEvent(1);
                if (e.key === 'ArrowRight') this.navigateEvent(-1);
            }
        });
        
        // Listener per cambio orientamento - ridisegna grafico
        window.addEventListener('orientationchange', () => {
            if (this.modal.style.display === 'block') {
                setTimeout(() => this.updateChart(), 300);
            }
        });
        
        // Listener per resize - ridisegna grafico
        window.addEventListener('resize', () => {
            if (this.modal.style.display === 'block') {
                setTimeout(() => this.updateChart(), 100);
            }
        });
    }
    
    open(eventElement, eventsList) {
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
        
        document.getElementById('modalTitle').textContent = 
            `${eventType} - ${eventName} (${new Date(this.currentEvent.timestamp * 1000).toLocaleString()})`;
        
        this.updateNavigationButtons();
        this.modal.style.display = 'block';
        
        this.refreshData();
    }
    
    close() {
        this.modal.style.display = 'none';
        if (this.chart) {
            this.chart.destroy();
            this.chart = null;
        }
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
        document.getElementById('prevEvent').disabled = this.currentEventIndex >= this.events.length - 1;
        document.getElementById('nextEvent').disabled = this.currentEventIndex <= 0;
    }
    
    refreshData() {
        if (!this.currentEvent) return;
        
        this.showLoading(true);
        const duration = document.querySelector('.btn-duration.active')?.dataset.duration || '10';
        
        this.rawData = null;
        this.metadata = null;
        this.processedData = null;
        
        const command = `get_buffer_window:${this.currentEvent.timestamp}:${duration}`;
        if (window.ws && window.ws.readyState === WebSocket.OPEN) {
            window.ws.send(command);
            this.showStatus(`Richiesta dati: ${duration}s attorno all'evento...`);
        } else {
            this.showStatus('WebSocket non connesso', true);
            this.showLoading(false);
        }
    }
    
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
        
        for (let i = 0; i < this.rawData.length; i++) {
            const packed = this.rawData[i];
            
            const infrared = (packed >> 0) & 0x1;
            const detect = (packed >> 1) & 0x1;
            const door_open = (packed >> 2) & 0x1;
            const newcode = (packed >> 3) & 0x1;
            const temp_closed = (packed >> 4) & 0x1;
            const manual_open = (packed >> 5) & 0x1;
            const authorized = (packed >> 6) & 0x1;
            const servo_moving = (packed >> 7) & 0x1;
            const rawAngle = (packed >> 8) & 0xFFF;
            const event_duration = (packed >> 20) & 0xFF;
            
            const angleDegrees = (rawAngle / 4095) * 360;
            const timeSeconds = i * 0.1;
            
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
                event_duration: event_duration / 10.0
            });
        }
        
        this.processedData = samples;
        console.log(`Processati ${samples.length} campioni`);
    }
    
    updateChart() {
        if (!this.processedData) return;
        
        const ctx = document.getElementById('eventChart').getContext('2d');
        
        if (this.chart) {
            this.chart.destroy();
            this.chart = null;
        }
        
        // Determina se siamo in landscape
        const isLandscape = window.matchMedia("(max-width: 768px) and (orientation: landscape)").matches;
        
        // Prepara datasets - DETTAGLI SEMPRE VISIBILI
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
            },
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
        ];
        
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
        
        // Configurazione legenda adaptive
        const legendConfig = isLandscape ? {
            display: this.legendVisible,
            position: 'chartArea',
            align: 'start',
            labels: {
                boxWidth: 8,
                padding: 6,
                font: { size: 9 }
            }
        } : {
            display: true,
            position: 'top',
            labels: {
                boxWidth: 12,
                padding: 15,
                font: { size: 11 }
            }
        };
        
        // Plugin custom per box semi-trasparente della legenda in landscape
        const legendBackgroundPlugin = isLandscape ? {
            id: 'legendBackground',
            beforeDraw: (chart) => {
                const legend = chart.legend;
                if (legend && legend.legendHitBoxes && legend.legendHitBoxes.length > 0) {
                    const ctx = chart.ctx;
                    const hitBoxes = legend.legendHitBoxes;

                    // Calcola il box che contiene tutti gli elementi della legenda
                    let minX = hitBoxes[0].left;
                    let minY = hitBoxes[0].top;
                    let maxX = hitBoxes[0].left + hitBoxes[0].width;
                    let maxY = hitBoxes[0].top + hitBoxes[0].height;

                    hitBoxes.forEach(box => {
                        minX = Math.min(minX, box.left);
                        minY = Math.min(minY, box.top);
                        maxX = Math.max(maxX, box.left + box.width);
                        maxY = Math.max(maxY, box.top + box.height);
                    });

                    // Aggiungi padding
                    const padding = 8;
                    minX -= padding;
                    minY -= padding;
                    maxX += padding;
                    maxY += padding;

                    // Disegna il box di sfondo
                    ctx.save();
                    ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
                    ctx.strokeStyle = 'rgba(0, 0, 0, 0.2)';
                    ctx.lineWidth = 1;
                    ctx.fillRect(minX, minY, maxX - minX, maxY - minY);
                    ctx.strokeRect(minX, minY, maxX - minX, maxY - minY);
                    ctx.restore();
                }
            }
        } : null;

        // Configurazione Chart.js
        this.chart = new Chart(ctx, {
            type: 'line',
            data: { datasets },
            plugins: legendBackgroundPlugin ? [legendBackgroundPlugin] : [],
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    intersect: false,
                    mode: 'nearest'
                },
                plugins: {
                    title: {
                        display: false  // TITOLO RIMOSSO
                    },
                    legend: legendConfig
                },
                scales: {
                    x: {
                        type: 'linear',
                        title: {
                            display: true,
                            text: 'Tempo (secondi)',
                            font: { size: isLandscape ? 10 : 12 }
                        },
                        grid: {
                            color: 'rgba(0,0,0,0.1)'
                        },
                        ticks: {
                            font: { size: isLandscape ? 9 : 11 }
                        }
                    },
                    angleAxis: {
                        type: 'linear',
                        position: 'left',
                        title: {
                            display: true,
                            text: 'Angolo Porta (°)',
                            font: { size: isLandscape ? 10 : 12 }
                        },
                        min: 0,
                        max: 360,
                        grid: {
                            color: 'rgba(52, 152, 219, 0.2)'
                        },
                        ticks: {
                            color: '#3498db',
                            font: { size: isLandscape ? 9 : 11 }
                        }
                    },
                    digitalAxis: {
                        type: 'linear',
                        position: 'right',
                        title: {
                            display: true,
                            text: 'Segnali',
                            font: { size: isLandscape ? 10 : 12 }
                        },
                        min: 0,
                        max: 360,
                        grid: {
                            display: false
                        },
                        ticks: {
                            color: '#7f8c8d',
                            font: { size: isLandscape ? 9 : 11 },
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
                    }
                }
            }
        });
    }
    
    showLoading(show) {
        document.getElementById('loadingOverlay').style.display = show ? 'flex' : 'none';
    }
    
    showStatus(message, isError = false) {
        const statusElement = document.getElementById('statusInfo');
        if (statusElement) {
            statusElement.textContent = message;
            statusElement.style.color = isError ? '#e74c3c' : '#666';
        }
    }
}

// Inizializza il modal quando il DOM è pronto
document.addEventListener('DOMContentLoaded', function() {
    window.eventGraphModal = new EventGraphModal();
});

// Funzione globale per aprire il modal
function openEventGraph(eventElement) {
    const eventRows = Array.from(document.querySelectorAll('#logBody tr'));
    window.eventGraphModal.open(eventElement, eventRows);
}
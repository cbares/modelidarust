//! Télécommande en terminal pour modelidar, via la liaison série USB (ST-Link VCP).
//!
//! L'appli interroge la carte avec `?` 10 fois par seconde, ce qui sert aussi de
//! signe de vie (le firmware arrête les moteurs après 500 ms sans trame USB).
//!
//! Lancement : `cargo run --release -- [--port /dev/ttyACM0]` depuis tools/teleop.

mod protocol;

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant};

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Paragraph};
use ratatui::{DefaultTerminal, Frame};

use protocol::{Reply, ZERO, parse, speed_frame};

const BAUDRATE: u32 = 115_200;
/// Pas codeur par tour de roue (src/tasks/odometry.rs).
const TICKS_PER_REV: f32 = 544.0;
/// Période d'interrogation, bien en dessous des 500 ms du firmware.
const POLL_PERIOD: Duration = Duration::from_millis(100);
/// Sans réponse pendant ce délai, la liaison est affichée perdue.
const LINK_LOST: Duration = Duration::from_millis(500);
/// Incrément de consigne par appui (10 %).
const STEP: i32 = 50;

/// Messages du fil de lecture série.
enum Incoming {
    Line(String),
    Closed(String),
}

struct App {
    port_name: String,
    throttle: i32,
    steer: i32,
    last_sent: String,
    state: Option<Reply>,
    last_reply: Option<Instant>,
    /// Lignes hors protocole (ex. « MCU started »), ou erreur de liaison.
    other_lines: VecDeque<String>,
}

impl App {
    fn note(&mut self, line: String) {
        self.other_lines.push_back(line);
        while self.other_lines.len() > 3 {
            self.other_lines.pop_front();
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port_name = port_from_args()?;
    let port = serialport::new(&port_name, BAUDRATE)
        .timeout(Duration::from_millis(50))
        .open()
        .map_err(|e| format!("ouverture de {port_name} impossible : {e}"))?;
    let reader = port.try_clone()?;
    let rx = spawn_reader(reader);

    let mut app = App {
        port_name,
        throttle: 0,
        steer: 0,
        last_sent: "?".into(),
        state: None,
        last_reply: None,
        other_lines: VecDeque::new(),
    };

    let mut terminal = ratatui::init();
    let result = run(&mut terminal, &mut app, port, &rx);
    ratatui::restore();
    result?;
    println!("Arrêt envoyé (/), à bientôt.");
    Ok(())
}

fn port_from_args() -> Result<String, String> {
    let mut args = std::env::args().skip(1);
    match (args.next().as_deref(), args.next()) {
        (Some("--port"), Some(port)) => Ok(port),
        (None, _) => default_port().ok_or_else(|| {
            "aucun port série trouvé : brancher la Nucleo ou préciser --port".to_string()
        }),
        _ => Err("usage : teleop [--port /dev/ttyACM0]".to_string()),
    }
}

/// Premier /dev/ttyACM* (ST-Link), sinon /dev/ttyUSB*.
fn default_port() -> Option<String> {
    let mut ports: Vec<String> = std::fs::read_dir("/dev")
        .ok()?
        .filter_map(|e| e.ok()?.file_name().into_string().ok())
        .filter(|n| n.starts_with("ttyACM") || n.starts_with("ttyUSB"))
        .map(|n| format!("/dev/{n}"))
        .collect();
    ports.sort_by_key(|p| (!p.contains("ttyACM"), p.clone()));
    ports.into_iter().next()
}

/// Lit le port en continu et renvoie les lignes reçues.
fn spawn_reader(mut port: Box<dyn serialport::SerialPort>) -> Receiver<Incoming> {
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        let mut buf = [0u8; 256];
        let mut pending = Vec::new();
        loop {
            match port.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => {
                    pending.extend_from_slice(&buf[..n]);
                    while let Some(i) = pending.iter().position(|&b| b == b'\n') {
                        let line: Vec<u8> = pending.drain(..=i).collect();
                        let line = String::from_utf8_lossy(&line).trim().to_string();
                        if !line.is_empty() && tx.send(Incoming::Line(line)).is_err() {
                            return;
                        }
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
                Err(e) => {
                    let _ = tx.send(Incoming::Closed(e.to_string()));
                    return;
                }
            }
        }
    });
    rx
}

fn run(
    terminal: &mut DefaultTerminal,
    app: &mut App,
    mut port: Box<dyn serialport::SerialPort>,
    rx: &Receiver<Incoming>,
) -> io::Result<()> {
    let mut next_poll = Instant::now();
    loop {
        terminal.draw(|f| draw(f, app))?;

        let timeout = next_poll.saturating_duration_since(Instant::now());
        let mut frame = None;
        if event::poll(timeout)?
            && let Event::Key(key) = event::read()?
            && key.kind == KeyEventKind::Press
        {
            match key.code {
                KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => break,
                KeyCode::Up => app.throttle = (app.throttle + STEP).min(ZERO),
                KeyCode::Down => app.throttle = (app.throttle - STEP).max(-ZERO),
                KeyCode::Right => app.steer = (app.steer + STEP).min(ZERO),
                KeyCode::Left => app.steer = (app.steer - STEP).max(-ZERO),
                KeyCode::Char(' ') => (app.throttle, app.steer) = (0, 0),
                KeyCode::Char('s') | KeyCode::Char('S') => {
                    (app.throttle, app.steer) = (0, 0);
                    frame = Some("/".to_string());
                }
                KeyCode::Char('b') | KeyCode::Char('B') => {
                    (app.throttle, app.steer) = (0, 0);
                    frame = Some("*".to_string());
                }
                _ => {}
            }
            if matches!(key.code, KeyCode::Up | KeyCode::Down | KeyCode::Left | KeyCode::Right | KeyCode::Char(' ')) {
                frame = Some(speed_frame(app.throttle, app.steer));
            }
        }

        // Une consigne vaut aussi interrogation : la carte répond avec son état.
        let now = Instant::now();
        if let Some(frame) = frame {
            port.write_all(frame.as_bytes())?;
            app.last_sent = frame;
            next_poll = now + POLL_PERIOD;
        } else if now >= next_poll {
            port.write_all(b"?")?;
            next_poll = now + POLL_PERIOD;
        }

        for msg in rx.try_iter() {
            match msg {
                Incoming::Line(line) => match parse(&line) {
                    Some(reply) => {
                        app.state = Some(reply);
                        app.last_reply = Some(Instant::now());
                    }
                    None => app.note(line),
                },
                Incoming::Closed(err) => app.note(format!("liaison fermée : {err}")),
            }
        }
    }
    port.write_all(b"/")?;
    port.flush()
}

/// Barre centrée pour une consigne dans [-ZERO, ZERO].
fn bar(value: i32) -> String {
    const HALF: i32 = 10;
    let n = (value.abs() * HALF + ZERO / 2) / ZERO;
    let cells: String = (-HALF..=HALF)
        .map(|i| match i {
            0 => '|',
            i if value > 0 && i > 0 && i <= n => '█',
            i if value < 0 && i < 0 && -i <= n => '█',
            _ => ' ',
        })
        .collect();
    format!("[{cells}]")
}

fn status_span(text: &str, ok: bool, warn: Color) -> Span<'static> {
    let color = if ok { Color::Green } else { warn };
    Span::styled(text.to_string(), Style::new().fg(color).add_modifier(Modifier::BOLD))
}

fn draw(frame: &mut Frame, app: &App) {
    let [header, asked, board, other, help] = Layout::vertical([
        Constraint::Length(3),
        Constraint::Length(5),
        Constraint::Length(8),
        Constraint::Length(4),
        Constraint::Min(0),
    ])
    .areas(frame.area());

    let alive = app.last_reply.is_some_and(|t| t.elapsed() < LINK_LOST);
    frame.render_widget(
        Paragraph::new(vec![
            Line::from(vec![
                "modelidar · télécommande   ".bold(),
                format!("{} @ {}", app.port_name, BAUDRATE).into(),
            ]),
            Line::from(vec![
                "Liaison : ".into(),
                status_span(if alive { "OK" } else { "pas de réponse" }, alive, Color::Red),
            ]),
        ]),
        header,
    );

    frame.render_widget(
        Paragraph::new(vec![
            Line::from(format!("avance     {:+5}  {}", app.throttle, bar(app.throttle))),
            Line::from(format!("direction  {:+5}  {}", app.steer, bar(app.steer))),
            Line::from(format!("dernière trame : {}", app.last_sent)),
        ])
        .block(Block::new().title("Demandé".underlined())),
        inset(asked),
    );

    let board_lines = match &app.state {
        None => vec![Line::from("en attente de réponse…")],
        Some(s) => {
            let (al, ar) = s.alpha;
            let wheel = |side: &str, w: &protocol::Wheel| {
                Line::from(format!(
                    "roue {side} : {:+9.1} pas/s  {:+6.2} tr/s   position {:+8}",
                    w.velocity,
                    w.velocity / TICKS_PER_REV,
                    w.position
                ))
            };
            let owner = match s.owner.as_str() {
                "USB" => "USB (ce terminal)",
                "RC" => "Radiocommande (USB ignoré)",
                "LOST" => "Radio perdue : moteurs arrêtés, USB bloqué",
                other => other,
            };
            let fault = match s.fault.as_str() {
                "-" => "aucun",
                "USB_TIMEOUT" => "PC muet trop longtemps : arrêt",
                "STALL" => "roue bloquée : revenir au neutre (espace)",
                other => other,
            };
            vec![
                Line::from(format!("consigne appliquée  G {al:+5}  {}", bar(al))),
                Line::from(format!("                    D {ar:+5}  {}", bar(ar))),
                wheel("G", &s.left),
                wheel("D", &s.right),
                Line::from(vec!["pilote : ".into(), status_span(owner, s.owner == "USB", Color::Yellow)]),
                Line::from(vec!["défaut : ".into(), status_span(fault, s.fault == "-", Color::Red)]),
            ]
        }
    };
    frame.render_widget(
        Paragraph::new(board_lines).block(Block::new().title("Carte".underlined())),
        inset(board),
    );

    let other_lines: Vec<Line> = app.other_lines.iter().map(|l| Line::from(format!("> {l}")).dim()).collect();
    frame.render_widget(Paragraph::new(other_lines), inset(other));

    frame.render_widget(
        Paragraph::new(vec![
            Line::from("↑ / ↓   avancer / reculer (±10 %)"),
            Line::from("← / →   tourner (±10 %)"),
            Line::from("espace  consigne neutre (réarme après un blocage)"),
            Line::from("s       arrêt roue libre (/)"),
            Line::from("b       freinage (*)"),
            Line::from("q       quitter (envoie un arrêt)"),
        ])
        .block(Block::new().title("Touches".underlined())),
        inset(help),
    );
}

/// Décalage de 2 colonnes pour le contenu des sections.
fn inset(area: Rect) -> Rect {
    Rect { x: area.x + 1, width: area.width.saturating_sub(1), ..area }
}

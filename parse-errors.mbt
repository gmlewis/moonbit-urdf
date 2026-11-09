///|
/// URDF parse diagnostics.

///|
#derive(Debug, Eq)
pub(all) struct UrdfParseErrorInfo {
  message : String
  line : Int32?
  col : Int32?
  snippet : String?
}

///|
pub suberror UrdfParseError {
  UrdfParseError(UrdfParseErrorInfo)
}

///|
pub(all) enum ParseUrdfResult {
  Ok(Robot)
  Err(UrdfParseErrorInfo)
}

///|
/// Formats a parse error as `@line:col: message`, plus an optional snippet.
pub fn format_urdf_parse_error(info : UrdfParseErrorInfo) -> String {
  let pos_prefix = match (info.line, info.col) {
    (Some(l), Some(c)) => "@" + l.to_string() + ":" + c.to_string() + ": "
    _ => ""
  }
  let header = pos_prefix + info.message
  match info.snippet {
    None => header
    Some(s) => {
      let lines = s.replace_all(old="\n", new="\n  ")
      header + "\nSnippet:\n  " + lines
    }
  }
}

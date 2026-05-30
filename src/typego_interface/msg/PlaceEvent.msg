# A place-graph lifecycle event emitted by Phase-6 matching, surfaced so a
# semantic-evidence scheduler can react (label new regions, GC dead ones,
# re-verify split/merged ones).
string kind          # birth | split | merge | death | refresh | provisional_cleared
string[] old_ids
string[] new_ids

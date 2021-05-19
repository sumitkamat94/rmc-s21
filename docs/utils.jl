function hfun_bar(vname)
  val = Meta.parse(vname[1])
  return round(sqrt(val), digits=2)
end

function hfun_m1fill(vname)
  var = vname[1]
  return pagevar("index", var)
end

function lx_baz(com, _)
  # keep this first line
  brace_content = Franklin.content(com.braces[1]) # input string
  # do whatever you want here
  return uppercase(brace_content)
end

# Re-define this function from Franklin.jl, gets overwritten
# Naturally, utils.jl needs to be included later for this overwrite to work.
function hfun_taglist()::String
    tag = locvar(:fd_tag)::String

    c = IOBuffer()
    write(c, "<ul>")

    rpaths = globvar("fd_tag_pages")[tag]
    sorter(p) = begin
        pvd = pagevar(p, "date")
        if isnothing(pvd)
            return Date(Dates.unix2datetime(stat(p * ".md").ctime))
        end
        return pvd
    end
    sort!(rpaths, by=sorter, rev=true)

    #(:hfun_list, "tag: $tag, loop over $rpaths") |> logger

    for rpath in rpaths
        title = pagevar(rpath, "title")
        if isnothing(title)
            title = "/$rpath/"
        end
        wurl = globvar("website_url")
        url = get_url(rpath)
        write(c, "<li><a href=\"$wurl$url\">$title</a></li>")
    end
    write(c, "</ul>")
    return String(take!(c))
end

@delay function hfun_taglistall()::String
    all_tags = globvar("fd_page_tags")
    (all_tags === nothing) && return ""
    all_tags = union(values(all_tags)...)
    c = IOBuffer()
    write(c, "Tags: ")
    for t in all_tags
        write(c, "<a href = \"/tag/$t/\">$t</a>, ")
    end
    return String(take!(c))
end

function hfun_reviewer_list()::String
    c = IOBuffer()
    revlist = locvar("reviewers")
    for x in revlist
        write(c, "$x, ")
    end
    return String(take!(c))
end

function hfun_fill_capital(params)::String
    t = locvar(:fd_tag)
    return uppercasefirst(t)
end

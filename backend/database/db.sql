--
-- PostgreSQL database dump
--

-- Dumped from database version 17.4 (Debian 17.4-1.pgdg120+2)
-- Dumped by pg_dump version 17.4 (Debian 17.4-1.pgdg120+2)

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET transaction_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: projects; Type: TABLE; Schema: public; Owner: user-dev
--

CREATE TABLE public.projects (
    id character varying(100) NOT NULL,
    name character varying(100) NOT NULL,
    creator integer NOT NULL,
    last_modified timestamp with time zone NOT NULL,
    size bigint NOT NULL
);


ALTER TABLE public.projects OWNER TO "user-dev";

--
-- Name: projects projects_pkey; Type: CONSTRAINT; Schema: public; Owner: user-dev
--

ALTER TABLE ONLY public.projects
    ADD CONSTRAINT projects_pkey PRIMARY KEY (id);


--
-- PostgreSQL database dump complete
--

